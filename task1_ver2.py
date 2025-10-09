# task1_forward_stop_hambot.py
# Lab 2 – Task 1 (HamBot): PID forward wall stop using ONE front LIDAR ray (idx=180)

import time, math
from robot_systems.robot import HamBot

# ---- Small, simple knobs ----
TARGET     = 0.50        # [m] stop distance
DEADBAND   = 0.02        # [m] stop when |error| <= 2 cm
DT         = 0.032       # [s] control period
KP, KI, KD = 1.0, 0.05, 0.10   # PID gains

# Safe linear speed limits (m/s)
V_FWD_MAX  = 0.35
V_REV_MAX  = -0.12

# LIDAR: single front sample (per lab spec)
FRONT_IDX  = 180

# Convert m/s -> RPM only at the final send (HamBot API expects RPM)
R_WHEEL    = 0.045
RPM_MAX    = 75.0
def mps_to_rpm(v_mps: float) -> float:
    rpm = (v_mps / (2.0 * math.pi * R_WHEEL)) * 60.0
    return max(-RPM_MAX, min(RPM_MAX, rpm))

def clamp(v, lo, hi): return max(lo, min(hi, v))

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    print(f"\n[Task1/HamBot] ONE-RAY PID → target={TARGET:.2f} m | dt={DT:.3f}s | Kp={KP} Ki={KI} Kd={KD}")

    i_acc = 0.0
    e_prev = 0.0

    try:
        while True:
            t0 = time.time()

            # 1) Read one front ray (fallback to a large value if invalid)
            scan = bot.get_range_image()
            if scan == -1 or len(scan) <= FRONT_IDX:
                bot.stop_motors()
                print("[WARN] LIDAR not ready; waiting…")
                time.sleep(0.2)
                continue
            y = float(scan[FRONT_IDX])
            if not math.isfinite(y) or y <= 0.0:
                y = 10.0  # simple fallback

            # 2) Error: positive when far (go forward), negative when close (back up)
            e = y - TARGET

            # 3) Stop exactly within deadband: zero command + reset integral
            if abs(e) <= DEADBAND:
                bot.stop_motors() 
                i_acc = 0.0
            else:
                # PID (discrete, minimal)
                d = (e - e_prev) / DT
                u = KP*e + KI*i_acc + KD*d

                # Saturate to linear speed limits
                v_cmd = clamp(u, V_REV_MAX, V_FWD_MAX)

                # Simple anti-windup: integrate only when not saturated
                if v_cmd == u:
                    i_acc += e * DT

            # Safety: hard brake if absurdly close
            if y < 0.10:
                bot.stop_motors() 
                i_acc = 0.0

            # 4) Send equal wheel speeds (straight); convert m/s → RPM only here
            rpm = mps_to_rpm(v_cmd)
            bot.set_left_motor_speed(rpm)    # HamBot handles left inversion internally
            bot.set_right_motor_speed(rpm)

            # 5) Telemetry
            print(f"[Task1] front={y:5.2f} m | e={e:+6.3f} | v={v_cmd:+.3f} m/s")

            e_prev = e

            # Keep loop near DT
            elapsed = time.time() - t0
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n[Task1] Stopping…")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    main()
