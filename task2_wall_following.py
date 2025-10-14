# wall_follow_hambot_curve_turn.py
# -------------------------------------------------------------
# HamBot – Task 2 wall following (cautious, curved turns) with selectable side
# - Uses HamBot.get_range_image() (360 beams: 0=back, 90=left, 180=front, 270=right)
# - Normalizes each scan to meters (auto-detect mm/cm/m)
# - PD on side distance for steering; P(+D) on forward speed
# - Curved, sensor-driven cornering (bounded ≤45° arcs, edge hugging)
# - Gentle wrap-search if wall vanishes
# - Extra diagonals (±45°) and inner-wheel boost when wall is “lost”
# - FOLLOW_SIDE selectable via CLI: python file.py [left|right]
# -------------------------------------------------------------

import sys, time, math
from robot_systems.robot import HamBot

# === Behavior toggles & targets ===
FOLLOW_SIDE   = "left"   # will be overridden by CLI args below if provided
DT            = 0.02     # control period (s)
TARGET_SIDE   = 0.30     # desired distance to side wall (m)
STOP_HARD     = 0.25     # emergency stop (front) (m)
TURN_TRIGGER  = 0.45     # start curved turn when front < this (m)
WALL_GONE     = 0.60     # wrap-search if side > this (m)

# === Cautious speeds (RPM) ===
MAX_RPM       = 75.0
CRUISE_RPM    = 28.0     # slower cruise for precision

# === Gains (tuned gently) ===
SIDE_KP       = 1.2
SIDE_KD       = 0.25
FWD_KP        = 1.0
FWD_KD        = 0.05

# === Curved turn params (replaces fixed 90°) ===
ARC_FAST_RPM  = 22.0     # outer wheel during arc (cautious)
ARC_SLOW_RPM  =  6.0     # inner wheel during arc (cautious)
ARC_MAX_SEC   = 1.2      # fail-safe timeout for arc
FRONT_CLEAR   = 0.50     # consider front “clear” when above this
SIDE_BLOCK    = 0.22     # “too close” to the side wall
ANGLE_LIMIT   = math.pi / 4.0   # ≤ 45° per arc

# === Wrap-search (gentle spin to reacquire wall) ===
SEARCH_RPM    = 22.0
SEARCH_SEC    = 0.50

# === LIDAR sectors (indices) ===
FRONT_S = slice(160, 200)   # ≈ 180°
LEFT_S  = slice( 90, 116)   # ≈  90°
RIGHT_S = slice(244, 280)   # ≈ 270°
BACK_S  = slice(355, 360)   # ≈   0°/360°

# Diagonal beams to disambiguate corners
LDIAG_IDX = 135             # front-left diagonal (≈+45° from 90)
RDIAG_IDX = 225             # front-right diagonal (≈-45° from 270)

# Extra 45° diagonals (requested)
L45_IDX   = 120             # between left (90) and front-left diag (135)
R45_IDX   = 240             # between right (270) and front-right diag (225)

# -------------------------------------------------------------
# HamBot Physical Specifications (from datasheet)
# -------------------------------------------------------------
WHEEL_RADIUS_M            = 0.045   # 90 mm diameter / 2
TRACK_WIDTH_M             = 0.184   # axle-to-axle distance
MAX_WHEEL_SPEED_MPS       = 0.81
MAX_WHEEL_SPEED_RAD_S     = 18.0

def rpm_to_radps(rpm):
    return rpm * (2.0 * math.pi / 60.0)

def robot_yaw_rate_rad_s(l_rpm, r_rpm):
    """Approximate robot yaw rate (rad/s) given left/right wheel RPM."""
    wl = rpm_to_radps(l_rpm)
    wr = rpm_to_radps(r_rpm)
    return (WHEEL_RADIUS_M * (wr - wl)) / TRACK_WIDTH_M

def ramp(prev, target, step):
    """Softly ramp toward target to make turns smoother."""
    if target > prev:  return min(target, prev + step)
    if target < prev:  return max(target, prev - step)
    return prev

# -------------------------------------------------------------
# Helpers
# -------------------------------------------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def safe_min(vals, fallback=float("inf")):
    """Min ignoring <=0/NaN/Inf; fallback if none valid."""
    good = []
    for v in vals:
        try:
            fv = float(v)
        except:
            continue
        if fv > 0.0 and math.isfinite(fv):
            good.append(fv)
    return min(good) if good else fallback

def beam(scan, idx, win=0):
    """Return min in a tiny window around a single index (conservative)."""
    n = len(scan)
    if n == 0:
        return float("inf")
    if win <= 0:
        v = float(scan[idx % n])
        return v if (v > 0.0 and math.isfinite(v)) else float("inf")
    vals = []
    for k in range(-win, win + 1):
        v = float(scan[(idx + k) % n])
        if v > 0.0 and math.isfinite(v):
            vals.append(v)
    return min(vals) if vals else float("inf")

def _median(xs):
    ys = sorted(xs)
    return ys[len(ys)//2] if ys else float("inf")

def normalize_scan_to_m(scan):
    """Auto-detect units per frame and return distances in meters."""
    samples = [v for v in scan if v > 0.0]
    if not samples:
        return [float("inf")] * len(scan)
    med = _median(samples[:min(200, len(samples))])
    if     med > 50.0: s = 0.001  # mm
    elif   med > 5.0:  s = 0.01   # cm
    else:              s = 1.0    # meters
    out = []
    for v in scan:
        if v <= 0.0:
            out.append(float("inf"))
        else:
            m = v * s
            out.append(m if m < 20.0 else float("inf"))
    return out

# -------------------------------------------------------------
# CLI parsing for FOLLOW_SIDE
# -------------------------------------------------------------
def parse_follow_side_from_argv():
    global FOLLOW_SIDE
    # Accept: python file.py right   OR   python file.py --side right
    args = [a.lower() for a in sys.argv[1:]]
    if not args:
        return
    if args[0] in ("left", "right"):
        FOLLOW_SIDE = args[0]
        return
    if "--side" in args:
        i = args.index("--side")
        if i + 1 < len(args) and args[i+1] in ("left", "right"):
            FOLLOW_SIDE = args[i+1]

# -------------------------------------------------------------
# Main
# -------------------------------------------------------------
def main():
    parse_follow_side_from_argv()

    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    time.sleep(0.3)

    side_e_prev = 0.0
    fwd_e_prev  = 0.0

    print(f"[Task2] follow={FOLLOW_SIDE}, target_side={TARGET_SIDE:.2f} m")

    try:
        # One-time sanity
        scan0   = bot.get_range_image()
        scan0_m = normalize_scan_to_m(scan0)
        print("[SANITY] Front/Left/Right/Back (m):",
              round(safe_min(scan0_m[FRONT_S]), 3),
              round(safe_min(scan0_m[LEFT_S ]), 3),
              round(safe_min(scan0_m[RIGHT_S]), 3),
              round(safe_min(scan0_m[BACK_S ]), 3))

        while True:
            scan = bot.get_range_image()
            if scan == -1 or len(scan) < 360:
                print("[WARN] LIDAR not ready.]")
                bot.stop_motors()
                time.sleep(0.1)
                continue

            scan_m  = normalize_scan_to_m(scan)
            front_d = safe_min(scan_m[FRONT_S])
            left_d  = safe_min(scan_m[LEFT_S ])
            right_d = safe_min(scan_m[RIGHT_S])

            # extra diagonals (used for decisions & edge hugging)
            ldiag   = beam(scan_m, LDIAG_IDX, win=2)
            rdiag   = beam(scan_m, RDIAG_IDX, win=2)
            l45     = beam(scan_m, L45_IDX,   win=1)
            r45     = beam(scan_m, R45_IDX,   win=1)

            side_d  = left_d if FOLLOW_SIDE == "left" else right_d

            # --- Emergency stop ---
            if front_d <= STOP_HARD:
                bot.stop_motors()
                print(f"[STOP] front={front_d:.2f} m")
                time.sleep(0.1)
                continue

            # --- Curved cornering (bounded ≤45° & edge-hugging) ---
            if front_d < TURN_TRIGGER:
                # Turn decision, mirrored for side
                if FOLLOW_SIDE == "left":
                    if front_d < SIDE_BLOCK and left_d < SIDE_BLOCK:
                        turn_dir = "RIGHT"  # classic right-corner
                    elif (left_d > TARGET_SIDE*1.8) or (ldiag > TARGET_SIDE*1.8) or (l45 > TARGET_SIDE*1.8):
                        turn_dir = "LEFT"   # new left edge appears; hug it
                    else:
                        left_open  = max(ldiag, l45)
                        right_open = max(rdiag, r45)
                        turn_dir = "LEFT" if left_open >= right_open else "RIGHT"
                else:  # FOLLOW_SIDE == "right"
                    if front_d < SIDE_BLOCK and right_d < SIDE_BLOCK:
                        turn_dir = "LEFT"
                    elif (right_d > TARGET_SIDE*1.8) or (rdiag > TARGET_SIDE*1.8) or (r45 > TARGET_SIDE*1.8):
                        turn_dir = "RIGHT"
                    else:
                        right_open = max(rdiag, r45)
                        left_open  = max(ldiag, l45)
                        turn_dir = "RIGHT" if right_open >= left_open else "LEFT"

                print(f"[ARC-{turn_dir}] F={front_d:.2f} L={left_d:.2f} R={right_d:.2f} "
                      f"Ld={ldiag:.2f} Rd={rdiag:.2f} L45={l45:.2f} R45={r45:.2f}")

                t0 = time.time()
                yaw_accum = 0.0
                cmd_L = 0.0
                cmd_R = 0.0

                while True:
                    # Sense during arc
                    sc   = bot.get_range_image()
                    sc_m = normalize_scan_to_m(sc)
                    f    = safe_min(sc_m[FRONT_S])
                    L    = safe_min(sc_m[LEFT_S ])
                    R    = safe_min(sc_m[RIGHT_S])
                    Sd   = L if FOLLOW_SIDE == "left" else R

                    # diagonals for hugging during the arc
                    ldiag_now = beam(sc_m, LDIAG_IDX, win=2)
                    rdiag_now = beam(sc_m, RDIAG_IDX, win=2)
                    l45_now   = beam(sc_m, L45_IDX,   win=1)
                    r45_now   = beam(sc_m, R45_IDX,   win=1)

                    # Adaptive arc speeds (smooth + edge-hug + wall-loss inner boost)
                    if (turn_dir == "LEFT" and FOLLOW_SIDE == "left") or (turn_dir == "RIGHT" and FOLLOW_SIDE == "right"):
                        # turning toward the followed side → inner = left when left-follow, inner = right when right-follow
                        if turn_dir == "LEFT":
                            target_L = ARC_SLOW_RPM
                            target_R = ARC_FAST_RPM
                            # losing left wall? boost inner (left), ease outer
                            if (not math.isfinite(Sd)) or (Sd > WALL_GONE) or (ldiag_now > WALL_GONE) or (l45_now > WALL_GONE):
                                target_L = min(MAX_RPM, target_L * 1.25)
                                target_R = max(0.0,    target_R * 0.95)
                        else:  # RIGHT with right-follow
                            target_L = ARC_FAST_RPM
                            target_R = ARC_SLOW_RPM
                            if (not math.isfinite(Sd)) or (Sd > WALL_GONE) or (rdiag_now > WALL_GONE) or (r45_now > WALL_GONE):
                                target_R = min(MAX_RPM, target_R * 1.25)
                                target_L = max(0.0,    target_L * 0.95)
                    else:
                        # turning away from followed side (classic opposite corner)
                        if turn_dir == "LEFT":
                            target_L = ARC_SLOW_RPM
                            target_R = ARC_FAST_RPM
                        else:
                            target_L = ARC_FAST_RPM
                            target_R = ARC_SLOW_RPM

                    # Smoothness based on front clearance (both directions)
                    softness = clamp((f - STOP_HARD) / max(FRONT_CLEAR - STOP_HARD, 1e-6), 0.0, 1.0)
                    if turn_dir == "LEFT":
                        target_L = target_L * (0.80 + 0.20 * softness)
                        target_R = target_R * (0.95 + 0.05 * softness)
                    else:
                        target_L = target_L * (0.90 + 0.10 * softness)
                        target_R = target_R * (0.65 + 0.35 * softness)

                    # Ramp toward targets
                    cmd_L = ramp(cmd_L, target_L, step=3.0)
                    cmd_R = ramp(cmd_R, target_R, step=3.0)
                    bot.set_left_motor_speed(cmd_L)
                    bot.set_right_motor_speed(cmd_R)

                    # Integrate yaw to cap ≤45° per arc
                    omega = robot_yaw_rate_rad_s(cmd_L, cmd_R)  # rad/s (approx)
                    yaw_accum += abs(omega) * DT

                    # Exit conditions (front clear AND side in band) OR angle cap OR timeout
                    done_front = (f > FRONT_CLEAR)
                    done_side  = (TARGET_SIDE*0.7 <= Sd <= TARGET_SIDE*1.7)
                    reached_angle = (yaw_accum >= ANGLE_LIMIT)
                    timeout = (time.time() - t0) > ARC_MAX_SEC

                    if (done_front and done_side) or reached_angle or timeout:
                        bot.stop_motors()
                        time.sleep(0.02)
                        break

                    time.sleep(DT)

                # reset small history
                side_e_prev = 0.0
                fwd_e_prev  = 0.0
                continue

            # --- Wall vanished (wrap-search) ---
            if side_d >= WALL_GONE or not math.isfinite(side_d):
                print(f"[WRAP] wall far: side={side_d:.2f} m → slow arc to reacquire")
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

            # --- Forward speed (P + small D) toward ~0.5 m front buffer ---
            fwd_e  = front_d - 0.50
            fwd_de = (fwd_e - fwd_e_prev) / DT
            fwd_e_prev = fwd_e
            v_fwd = CRUISE_RPM + 30.0 * (FWD_KP * fwd_e + FWD_KD * fwd_de)
            v_fwd = clamp(v_fwd, 0.0, CRUISE_RPM)

            # --- Side steering (PD): error = TARGET_SIDE - side_d ---
            side_e  = TARGET_SIDE - side_d
            side_de = (side_e - side_e_prev) / DT
            side_e_prev = side_e
            steer = SIDE_KP * side_e + SIDE_KD * side_de
            steer = -steer if FOLLOW_SIDE == "left" else +steer
            steer = clamp(steer, -0.35 * v_fwd, 0.35 * v_fwd)  # slightly tighter clamp for smoothness

            # --- Command motors ---
            left_rpm  = clamp(v_fwd + steer, -MAX_RPM, MAX_RPM)
            right_rpm = clamp(v_fwd - steer, -MAX_RPM, MAX_RPM)
            bot.set_left_motor_speed(left_rpm)
            bot.set_right_motor_speed(right_rpm)

            # --- Debug prints ---
            f = beam(scan_m, 180)
            l = beam(scan_m,  90)
            r = beam(scan_m, 270)
            print(f"[FOLLOW] side={side_d:.2f}  front={front_d:.2f}  vf={v_fwd:.1f}  "
                  f"steer={steer:+.1f}  L={left_rpm:.1f} R={right_rpm:.1f}")
            print(f"[SENSE]  F/L/R/B(m): {f:.2f} {l:.2f} {r:.2f} {beam(scan_m,0)::.2f}  "
                  f"Ld={ldiag:.2f} Rd={rdiag:.2f} L45={l45:.2f} R45={r45:.2f}")

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[Task2] stopped by user.")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    main()
