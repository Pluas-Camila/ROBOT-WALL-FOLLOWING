#Task1 - less distance slow down 

# e(t) = a-d
# Desired (d) is distance to the wall 
# actual (a) is the actual distance, gotten from the lidar. degrees 

# For lidar (a): do not take the average of the distance, Minimums will be best to use 
# [choose->123, 153] of these range do not take average 


# Phi(t) = k(p)*(e(t))
# Where phi is angular velocity and k(p) is a choosen variable k(p) = 3 is a good choice 

from robot_systems.robot import HamBot
import time
import math
import sys

# --- Initialize the robot ---
bot = HamBot()
DESIRED_DISTANCE = 1.0   # meters
KP = 1.8
#2.6                 # proportional gain
DT = 0.05                 # timestep (s)
MAX_SPEED = 16.0            # limit forward/backward velocity (m/s)
KI = 0.0
#0.2                 # integral gain (start small)
KD = 0.8                 # derivative gain


#robot = HamBot(lidar_enabled=True, camera_enabled=False)

def sat (v):
    if v > MAX_SPEED:
        return MAX_SPEED
    elif v < -MAX_SPEED:
        return -MAX_SPEED
    else:
        return v

def main():
# --- Main Control Loop ---
    integral = 0.0
    prev_error = 0.0
    while True:
        # 1. Read front-facing LIDAR (choose ~175–185 degrees for center)
        actual_distance = min(bot.get_range_image()[123:153])

        # 2. Compute error (difference from target distance)
        e = (actual_distance/100) - DESIRED_DISTANCE
        integral += e  * DT
        derivative = (e  - prev_error) / DT

        # 4. Compute PID output (forward velocity)
        forward_velocity = (KP * e) + (KI * integral) + (KD * derivative)
        forward_velocity = sat(forward_velocity)

        # 5. Apply equal motor velocities (straight motion)
        bot.set_left_motor_speed(forward_velocity)
        bot.set_right_motor_speed(forward_velocity)

        # 6. Print for debugging
        print(f"Distance: {actual_distance/100:.3f} m | Error: {e :.3f} | "
              f"V: {forward_velocity:.3f} | P: {KP*e :.3f} | I: {KI*integral:.3f} | D: {KD*derivative:.3f}")

        # 7. Save error and wait for next timestep
        prev_error = e 
        time.sleep(DT)

        # # 3. Compute forward velocity using proportional control
        # forward_velocity = sat(KP * e)

        # # 4. Apply equal left/right motor speeds for straight motion
        # bot.set_left_motor_speed(forward_velocity)
        # bot.set_right_motor_speed(forward_velocity)

        # # 5. Print readings for testing/verification
        # print(f"Distance: {actual_distance:.3f} m | Error: {e:.3f} | Forward Velocity: {forward_velocity:.3f}")

        # # 6. Wait for next timestep
        # time.sleep(DT)

if __name__ == "__main__":
    main()
