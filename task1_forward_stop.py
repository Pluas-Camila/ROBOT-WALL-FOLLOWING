# task1_forward_stop.py
# Lab 2 - Task 1: Forward PID (P-only) wall stop using LIDAR minimum
# ---------------------------------------------------------------
# - Uses the minimum distance in a front sector (no averaging).
# - Controls only forward speed (no steering).
# - If closer than the target, backs up gently until at setpoint.


import math
import time

# ===============================================================
#  PID CONFIGURATION
# ===============================================================
KP = 1.0        # Proportional gain
KI = 0.1        # Integral gain
KD = 0.5        # Derivative gain
TARGET_DIST = 0.5       # desired distance from wall (meters)
MAX_SPEED = 1.0         # maximum forward speed (m/s)
MIN_SPEED = -1.0        # maximum reverse speed (m/s)
DEADBAND = 0.02         # acceptable ±2 cm band

# LIDAR setup
FRONT_IDX = 180         # 180° = directly in front
SECTOR = 10             # check ±10° around front

# ===============================================================
#  PID CONTROLLER CLASS
# ===============================================================
class PIDController:
    def __init__(self, kp, ki, kd, i_clamp=None, d_smooth=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_d = 0.0
        self.i_clamp = i_clamp
        self.d_smooth = max(0.0, min(0.99, d_smooth))

    def update(self, error, dt):
        """Compute PID control signal."""
        # proportional
        p = self.kp * error

        # integral
        self.integral += error * dt
        i = self.ki * self.integral

        # derivative
        d = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # control output
        u = p + i + d
        return u

# ===============================================================
#  HELPER FUNCTIONS
# ===============================================================
def get_front_distance(robot):
    """
    Reads the LIDAR data and returns the minimum distance in a small
    angular window around the front direction (180°).
    """
    ranges = robot.get_lidar_range_image()
    start = FRONT_IDX - SECTOR
    end = FRONT_IDX + SECTOR
    front_window = ranges[start:end]
    return min(front_window)

def saturate(value, lo, hi):
    """Clamp motor velocity to physical limits."""
    return max(lo, min(hi, value))

# ===============================================================
#  MAIN CONTROL LOOP
# ===============================================================
def main():
    from MyRobot import MyRobot
    robot = MyRobot()

    pid = PIDController(KP, KI, KD)
    dt = 0.032  # timestep from assignment

    print(f"\n--- PID Forward Wall Stop ---")
    print(f"Kp={KP}, Ki={KI}, Kd={KD}, Target={TARGET_DIST} m")

    while robot.step() != -1:
        # 1️⃣ Measure actual distance
        actual = get_front_distance(robot)

        # 2️⃣ Compute error (desired - measured)
        error = TARGET_DIST - actual

        # 3️⃣ Compute PID output (forward speed)
        control = pid.update(error, dt)
        control = saturate(control, MIN_SPEED, MAX_SPEED)

        # 4️⃣ Apply deadband — stop when within ±2 cm
        if abs(error) < DEADBAND:
            control = 0.0

        # 5️⃣ Apply equal wheel speeds (forward only)
        robot.set_wheel_speeds(control, control)

        # 6️⃣ Print diagnostics
        print(f"Dist={actual:5.2f} m  Err={error:+6.3f}  u={control:+6.3f}")

        # optional: delay slightly for clarity
        time.sleep(dt)

if __name__ == "__main__":
    main()