# task1_forward_stop_hambot.py
# Lab 2 – Task 1 (HamBot): Minimal PID forward wall stop using ONLY the front LIDAR beam
# ---------------------------------------------------------------------------------------
# - Reads front distance at index 180.
# - PID controls straight-line speed (no steering).
# - Target stopping distance is 0.5 m.
# - All logic in m/s; convert to RPM only at the final send step.

import time, math
from robot_systems.robot import HamBot

# --- Simple knobs (keep it tiny) ---
TARGET     = 0.50   # [m] stop distance
DEADBAND   = 0.01   # [m] ±1 cm stop band
DT         = 0.032  # [s] control period
KP, KI, KD = 1.0, 0.05, 0.10  # PID gains (tweak here)

# --- Speed limits (safe & small; reverse capped low) ---
V_FWD_MAX  = 0.5   # [m/s]
V_REV_MAX  = 0  # [m/s] 

# --- LIDAR: single front sample index (per lab spec) ---
FRONT_IDX  = 180

# --- Robot wheel geometry for final conversion only ---
R_WHEEL = 0.045   # [m] radius
RPM_MAX = 75.0    # HamBot motor clamp (+/-)

def mps_to_rpm(v_mps: float) -> float:
    """Wheel linear speed [m/s] -> RPM (HamBot motor API needs RPM)."""
    rpm = (v_mps / (2.0 * math.pi * R_WHEEL)) * 60.0
    return max(-RPM_MAX, min(RPM_MAX, rpm))

def clamp(v, lo, hi):  # tiny helper
    return max(lo, min(hi, v))

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    print(f"\n[Task1/HamBot] PID to {TARGET:.2f} m | dt={DT:.3f}s | gains Kp={KP} Ki={KI} Kd={KD}")

    # PID state
    i_acc = 0.0
    e_prev = 0.0

    try:
        while True:
            t_loop = time.time()

            # 1) Read only the front LIDAR ray
            ranges = bot.get_range_image()
            if ranges == -1 or len(ranges) < 181:
                bot.stop_motors()
                print("[WARN] LIDAR not ready; waiting…")
                time.sleep(0.2)
                continue
            y = float(ranges[FRONT_IDX]) if ranges[FRONT_IDX] > 0 else 10.0  # simple fallback

            # 2) Error: positive when far (go forward), negative when close (back up)
            e = y - TARGET

            # 3) PID (discrete, minimal)
            i_acc += e * DT
            d = (e - e_prev) / DT
            v_cmd = KP*e + KI*i_acc + KD*d  # body forward speed [m/s]

            # 4) Deadband & clamp to safe forward/backward limits
            if abs(e) <= DEADBAND:
                v_cmd = 0.0
            v_cmd = clamp(v_cmd, V_REV_MAX, V_FWD_MAX)

            # 5) Send equal wheel speeds (straight), converting m/s 
            rpm = mps_to_rpm(v_cmd)
            bot.set_left_motor_speed(rpm)   # HamBot handles left inversion internally
            bot.set_right_motor_speed(rpm)

            # 6) Quick telemetry
            print(f"[Task1] front={y:5.2f} m | e={e:+6.3f} | v={v_cmd:+.3f} m/s")

            # 7) Keep loop near DT
            elapsed = time.time() - t_loop
            if elapsed < DT:
                time.sleep(DT - elapsed)

            e_prev = e

    except KeyboardInterrupt:
        print("\n[Task1] Stopping…")
    finally:
        bot.stop_motors()

if __name__ == "__main__":
    main()
