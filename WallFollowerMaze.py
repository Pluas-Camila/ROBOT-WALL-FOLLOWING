# wall_follow_hambot_fix_units.py
# -------------------------------------------------------------
# HamBot – Task 2 wall following w/ robust LIDAR unit normalization.
# - Normalizes each scan to meters (mm/cm/m)
# - Uses explicit sectors (0=back, 90=left, 180=front, 270=right)
# - P(+D) on side distance for steering; P on forward speed
# - Timed ~90° corner turn; wrap-search when wall is “gone”
# -------------------------------------------------------------

import time
from robot_systems.robot import HamBot

FOLLOW_SIDE = "left"         # "left" or "right"
DT = 0.02
MAX_RPM = 75.0
CRUISE_RPM = 45.0
TARGET_SIDE = 0.30           # meters
STOP_HARD = 0.25            # emergency stop distance (m)
TURN_TRIGGER = 0.45          # start corner turn when front < this (m)
WALL_GONE = 0.60             # treat as wall end if side > this (m)

# Gains
SIDE_KP = 1.2
SIDE_KD = 0.25
FWD_KP  = 1.0
FWD_KD  = 0.05

TURN_RPM = 50.0
TURN_SEC = 0.6
SEARCH_RPM = 25.0
SEARCH_SEC = 0.40

# Sector definitions (10°-ish windows)
FRONT_S = slice(160, 200)   # ~180°
LEFT_S  = slice( 90, 116)    # ~ 90°
RIGHT_S = slice(244, 280)    # ~270°
BACK_S  = slice(355, 360)    # ~  0°/360°

def clamp(x, lo, hi): return max(lo, min(hi, x))

def safe_min(vals):
    good = [v for v in vals if v > 0.0]
    return min(good) if good else float("inf")

def _median(xs):
    ys = sorted(xs)
    n = len(ys)
    if n == 0: return float("inf")
    return ys[n//2]

def normalize_scan_to_m(scan):
    """
    Return a list of distances in meters.
    Detects units per frame: if typical magnitudes look like
      - mm (100..5000) -> scale 0.001
      - cm (10..500)   -> scale 0.01
      - m  (0.05..10)  -> scale 1.0
    """
    # Filter to a subset of non-zero samples
    samples = [v for v in scan if v > 0.0]
    if not samples:
        return [float("inf")] * len(scan)

    med = _median(samples[: min(200, len(samples))])

    if med > 50.0:      # likely mm
        s = 0.001
    elif med > 5.0:     # likely cm
        s = 0.01
    else:               # likely meters
        s = 1.0

    out = []
    for v in scan:
        if v <= 0.0:
            out.append(float("inf"))
        else:
            m = v * s
            # clamp absurd values
            out.append(m if m < 20.0 else float("inf"))
    return out

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    time.sleep(0.3)

    side_e_prev = 0.0
    fwd_e_prev  = 0.0

    print(f"[Task2] Wall follow {FOLLOW_SIDE}, target side={TARGET_SIDE:.2f} m")

    try:
        # One-time: show a sanity line so you can confirm sectors & units
        scan0 = bot.get_range_image()
        scan0_m = normalize_scan_to_m(scan0)
        print("[SANITY] Front/Left/Right/Back (m):",
              round(safe_min(scan0_m[FRONT_S]),3),
              round(safe_min(scan0_m[LEFT_S]),3),
              round(safe_min(scan0_m[RIGHT_S]),3),
              round(safe_min(scan0_m[BACK_S]),3))

        while True:
            scan = bot.get_range_image()
            if scan == -1 or len(scan) < 360:
                print("[WARN] LIDAR not ready.")
                bot.stop_motors()
                time.sleep(0.1)
                continue

            scan_m = normalize_scan_to_m(scan)
            front_d = safe_min(scan_m[FRONT_S])
            left_d  = safe_min(scan_m[LEFT_S])
            right_d = safe_min(scan_m[RIGHT_S])
            # back_d  = safe_min(scan_m[BACK_S])  # not needed here

            side_d = left_d if FOLLOW_SIDE == "left" else right_d

            # Emergency stop
            if front_d <= STOP_HARD:
                bot.stop_motors()
                print(f"[STOP] front={front_d:.2f} m (emergency)")
                time.sleep(0.1)
                continue

            # Corner turn
            if front_d < TURN_TRIGGER:
                bot.stop_motors()
                time.sleep(0.05)
                print(f"[TURN] Corner: front={front_d:.2f} m")
                if FOLLOW_SIDE == "left":
                    bot.set_left_motor_speed(-TURN_RPM)
                    bot.set_right_motor_speed(+TURN_RPM)
                else:
                    bot.set_left_motor_speed(+TURN_RPM)
                    bot.set_right_motor_speed(-TURN_RPM)
                time.sleep(TURN_SEC)
                bot.stop_motors()
                time.sleep(0.05)
                side_e_prev = 0.0
                fwd_e_prev  = 0.0
                continue

            # Wall gone (open corner) → wrap-search
            if side_d >= WALL_GONE:
                print(f"[WRAP] Side too far: {side_d:.2f} m → search {FOLLOW_SIDE}")
                if FOLLOW_SIDE == "left":
                    bot.set_left_motor_speed(-SEARCH_RPM)
                    bot.set_right_motor_speed(+SEARCH_RPM)
                else:
                    bot.set_left_motor_speed(+SEARCH_RPM)
                    bot.set_right_motor_speed(-SEARCH_RPM)
                time.sleep(SEARCH_SEC)
                bot.stop_motors()
                time.sleep(0.02)
                continue

            # Forward speed (P + small D) toward ~0.5 m front buffer
            fwd_e = front_d - 0.50
            fwd_de = (fwd_e - fwd_e_prev) / DT
            fwd_e_prev = fwd_e
            v_fwd = CRUISE_RPM + 30.0 * (FWD_KP * fwd_e + FWD_KD * fwd_de)
            v_fwd = clamp(v_fwd, 0.0, CRUISE_RPM)

            # Side steering (P + D): too close → steer away
            side_e = TARGET_SIDE - side_d
            side_de = (side_e - side_e_prev) / DT
            side_e_prev = side_e
            steer = SIDE_KP * side_e + SIDE_KD * side_de
            steer = -steer if FOLLOW_SIDE == "left" else +steer

            # Keep steering bounded relative to forward
            steer = clamp(steer, -0.45 * v_fwd, 0.45 * v_fwd)

            left_rpm  = clamp(v_fwd + steer, -MAX_RPM, MAX_RPM)
            right_rpm = clamp(v_fwd - steer, -MAX_RPM, MAX_RPM)
            bot.set_left_motor_speed(left_rpm)
            bot.set_right_motor_speed(right_rpm)

            print(f"[FOLLOW] side={side_d:.2f} m  front={front_d:.2f} m  "
                  f"vf={v_fwd:.1f} rpm  steer={steer:+.1f} rpm  "
                  f"L={left_rpm:.1f} R={right_rpm:.1f}")

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[Task2] Stopped by user.")
    finally:
        bot.stop_motors()


if __name__ == "__main__":
    main()
