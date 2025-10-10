# task1_forward_stop.py
# Lab 2 – Task 1 (HamBot): ONE-RAY full PID forward wall stop with anti-windup
# ---------------------------------------------------------------------------
# - Reads one LIDAR ray (index 180) -> meters
# - PID drives straight speed (same-sign RPM to both wheels)
# - Derivative on measurement (less kick), anti-windup on saturation
# - Deadband + integral reset for crisp stop
# - Tiny EMA smoothing on the single ray (still "one ray" logic)

import time, math
from robot_systems.robot import HamBot

# ====== Lab targets ======
TARGET      = 1.0        # [m] stop distance
DEADBAND    = 0.01       # [m] ±1 cm

# ====== Timing ======
DT          = 0.032      # [s] control period

# ====== One-ray LIDAR ======
FRONT_IDX   = 180
MM_TO_M     = 1.0 / 1000.0

# ====== Motor limits ======
RPM_MAX     = 75.0       # HamBot clamp
# Optional: cap speed with softer limit to help PID tuning
RPM_SOFT    = 50.0

# ====== PID gains (units: RPM per m, RPM per (m*s), RPM per m/s) ======
# Start conservative; increase Kp for quicker approach; add small Ki for zero steady-state error; Kd to damp overshoot.
Kp_rpm      = 45.0       # proportional
Ki_rpm      = 6.0        # integral (small)
Kd_rpm      = 12.0       # derivative on measurement

# ====== Integral guard / smoothing ======
I_CLAMP     = 150.0      # limit absolute integral contribution (RPM-equivalent)
EMA_ALPHA   = 0.15       # 0=no smoothing, 1=raw; 0.1–0.2 good for noisy single ray

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    print(f"[Task1/PID] TARGET={TARGET:.2f} m | dt={DT:.3f}s | "
          f"Kp={Kp_rpm}, Ki={Ki_rpm}, Kd={Kd_rpm} (RPM units)")

    # PID state
    integ    = 0.0         # integral term (in “error-m*s” space, scaled by Ki_rpm when used)
    y_prev   = None        # previous distance [m]
    y_filt   = None        # EMA filtered distance
    first    = True

    try:
        while True:
            t0 = time.time()

            scan = bot.get_range_image()
            if scan == -1 or len(scan) <= FRONT_IDX:
                bot.stop_motors()
                print("[WARN] LIDAR not ready…")
                time.sleep(0.1)
                continue

            raw = float(scan[FRONT_IDX])
            if not math.isfinite(raw) or raw <= 0.0:
                bot.stop_motors()
                print("[WARN] invalid front sample; waiting…")
                time.sleep(0.05)
                continue

            # 1) distance in meters
            y = raw * MM_TO_M

            # 2) single-ray EMA (still one ray; just smoothed)
            if first:
                y_filt = y
                y_prev = y
                first = False
            else:
                y_filt = (1.0 - EMA_ALPHA) * y_filt + EMA_ALPHA * y

            # 3) error (far → positive; close → negative)
            e = y_filt - TARGET

            # 4) deadband → full brake + reset integral (prevents creep)
            if abs(e) <= DEADBAND:
                bot.stop_motors()
                integ = 0.0
                print(f"[STOP] y={y_filt:.3f} m | e={e:+.3f}")
                # small settle to avoid twitch in next loop
                time.sleep(DT)
                continue

            # 5) derivative on measurement (d(y)/dt) → subtract to damp approach
            dy = (y_filt - y_prev) / DT if y_prev is not None else 0.0
            y_prev = y_filt

            # 6) raw PID (in RPM)
            #    u = Kp*e + Ki*∫e dt − Kd*dy/dt   (minus sign on derivative of measurement)
            p_term = Kp_rpm * e
            i_term = Ki_rpm * integ
            d_term = -Kd_rpm * dy
            u_raw  = p_term + i_term + d_term

            # 7) saturation + anti-windup: integrate only when not saturated
            u_sat  = clamp(u_raw, -RPM_SOFT, RPM_SOFT)
            if abs(u_raw - u_sat) < 1e-6:
                # not saturated → allow integral to grow
                integ += e * DT
                # clamp the integral’s effect so it can’t dominate
                integ = clamp(integ, -I_CLAMP / max(Ki_rpm, 1e-9), I_CLAMP / max(Ki_rpm, 1e-9))
            else:
                # slightly bleed integral when saturated
                integ *= 0.98

            # 8) final hard clamp to motor range
            rpm_cmd = clamp(u_sat, -RPM_MAX, RPM_MAX)

            # 9) same sign to both motors → straight
            bot.set_left_motor_speed(rpm_cmd)
            bot.set_right_motor_speed(rpm_cmd)

            print(f"[RUN ] y={y_filt:.3f} m | e={e:+.3f} | P={p_term:+6.1f} I={i_term:+6.1f} D={d_term:+6.1f} | rpm={rpm_cmd:+6.1f}")

            # loop timing
            elapsed = time.time() - t0
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n[Task1/PID] Stopping…")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    main()
