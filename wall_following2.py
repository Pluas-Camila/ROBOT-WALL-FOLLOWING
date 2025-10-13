# wall_following2.py
import time, math
from robot_systems.robot import HamBot

# ---------------- Config ----------------
FOLLOW_SIDE   = "left"        # "left" or "right"
DT            = 0.02          # control period (s)
CRUISE_RPM    = 25.0
MAX_RPM       = 75.0

TARGET_SIDE   = 0.30
TURN_TRIGGER  = 0.45
STOP_HARD     = 0.25
WALL_GONE     = 0.60

SIDE_KP       = 1.2
SIDE_KD       = 0.25
FWD_KP        = 1.0
FWD_KD        = 0.05

TURN_RPM      = 40.0
TURN_SEC      = 0.60
COOLDOWN_SEC  = 0.25

FRONT_IDX = 180   # 0=back, 90=left, 180=front, 270=right
LEFT_IDX  = 90
RIGHT_IDX = 270

def clamp(x, lo, hi): return max(lo, min(hi, x))

def _median(xs):
    ys = [v for v in xs if v > 0]
    if not ys: return float("inf")
    ys.sort()
    return ys[len(ys)//2]

def normalize_scan_to_m(scan):
    vals = [v for v in scan if v and v > 0]
    if not vals:
        return [float("inf")] * len(scan)
    med = _median(vals[:min(200, len(vals))])
    scale = 0.001 if med > 50 else (0.01 if med > 5 else 1.0)
    out = []
    for v in scan:
        if v and v > 0:
            m = v * scale
            out.append(m if m < 20 else float("inf"))
        else:
            out.append(float("inf"))
    return out

def beam_min(scan_m, idx, win=16):
    n = len(scan_m)
    vals = []
    for k in range(-win, win+1):
        v = float(scan_m[(idx+k) % n])
        if v > 0 and math.isfinite(v):
            vals.append(v)
    return min(vals) if vals else float("inf")

def run():
    # --- create robot ---
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    time.sleep(0.3)

    # --- sanity: can we read lidar? ---
    raw = bot.get_range_image()
    if raw == -1 or not isinstance(raw, (list, tuple)) or len(raw) < 360:
        print("[FATAL] LIDAR not available (get_range_image() returned invalid). Exiting.")
        bot.stop_motors()
        return

    print(f"[Task2] follow={FOLLOW_SIDE}, target_side={TARGET_SIDE:.2f} m")

    # state
    mode = "FOLLOW"
    last_mode_change = time.time()
    side_err_prev = 0.0
    fwd_err_prev  = 0.0

    try:
        while True:
            raw = bot.get_range_image()
            if raw == -1 or len(raw) < 360:
                print("[WARN] LIDAR not ready; stopping.")
                bot.stop_motors()
                time.sleep(0.1)
                continue

            scan_m  = normalize_scan_to_m(raw)
            front_d = beam_min(scan_m, FRONT_IDX, win=1)
            left_d  = beam_min(scan_m, LEFT_IDX,  win=1)
            right_d = beam_min(scan_m, RIGHT_IDX, win=1)
            side_d  = left_d if FOLLOW_SIDE == "left" else right_d

            now = time.time()
            cooling = (now - last_mode_change) < COOLDOWN_SEC

            # emergency stop
            if front_d <= STOP_HARD:
                bot.stop_motors()
                print(f"[STOP] front={front_d:.3f}  left={left_d:.3f}  right={right_d:.3f}")
                time.sleep(DT)
                continue

            if mode == "FOLLOW":
                # corner?
                if (front_d < TURN_TRIGGER) and not cooling:
                    mode = "TURN90"
                    last_mode_change = now
                    bot.stop_motors()
                    print(f"[TURN->] front={front_d:.3f}  side={side_d:.3f}")
                    continue

                # follow: forward buffer + side steering
                fwd_err = front_d - 0.50
                fwd_de  = (fwd_err - fwd_err_prev) / DT
                fwd_err_prev = fwd_err
                v_fwd = CRUISE_RPM + 30.0 * (FWD_KP * fwd_err + FWD_KD * fwd_de)
                v_fwd = clamp(v_fwd, 0.0, CRUISE_RPM)

                side_err = TARGET_SIDE - side_d
                side_de  = (side_err - side_err_prev) / DT
                side_err_prev = side_err
                steer = SIDE_KP * side_err + SIDE_KD * side_de

                # sign so that for LEFT-follow, too close => steer right (CW)
                steer *= (-1.0 if FOLLOW_SIDE == "left" else +1.0)
                steer = clamp(steer, -0.45 * v_fwd, 0.45 * v_fwd)

                L = clamp(v_fwd - steer, -MAX_RPM, MAX_RPM)
                R = clamp(v_fwd + steer, -MAX_RPM, MAX_RPM)
                bot.set_left_motor_speed(L)
                bot.set_right_motor_speed(R)

                print(f"[FOLLOW] F={front_d:.3f}  Lft={left_d:.3f}  Rgt={right_d:.3f}  "
                      f"side={side_d:.3f}  v={v_fwd:5.1f}  steer={steer:+5.1f}  M=({L:5.1f},{R:5.1f})")

            elif mode == "TURN90":
                # For LEFT follow, turn RIGHT to keep wall on left
                cw = (FOLLOW_SIDE == "left")
                L = -TURN_RPM if cw else +TURN_RPM
                R = +TURN_RPM if cw else -TURN_RPM
                bot.set_left_motor_speed(L)
                bot.set_right_motor_speed(R)
                print(f"[TURN90] cmd=({L:.1f},{R:.1f}) for {TURN_SEC:.2f}s  "
                      f"F={front_d:.3f} side={side_d:.3f}")
                time.sleep(TURN_SEC)
                bot.stop_motors()
                time.sleep(0.05)
                side_err_prev = 0.0
                fwd_err_prev  = 0.0
                mode = "FOLLOW"
                last_mode_change = time.time()
                print("[TURN90->FOLLOW]")

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[Task2] Stopped by user.")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    run()
