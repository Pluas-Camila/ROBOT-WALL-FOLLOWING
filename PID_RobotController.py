#Task1 - less distance slow down 

# e(t) = a-d
# Desired (d) is distance to the wall 
# actual (a) is the actual distance, gotten from the lidar. degrees 

# For lidar (a): do not take the average of the distance, Minimums will be best to use 
# [choose->123, 153] of these range do not take average 


# Phi(t) = k(p)*(e(t))
# Where phi is angular velocity and k(p) is a choosen variable k(p) = 3 is a good choice 


# Constants
DESIRED_DISTANCE = 1.0   # meters
KP = 3.0                 # proportional gain

while robot.is_running():
    # 1. Get LiDAR scan (list or numpy array)
    lidar = robot.get_lidar_data()
    
    # 2. Select front range [123, 153]
    front_window = lidar[123:154]  # Python slice includes 123 to 153
    actual_distance = min(front_window)

    # 3. Compute error
    error = actual_distance - DESIRED_DISTANCE

    # 4. Compute control output
    phi = KP * error   # positive = move forward, negative = move backward

    # 5. Optional: clamp velocity
    phi = max(min(phi, 1.0), -1.0)

    # 6. Apply forward velocity (no lateral influence)
    robot.set_velocity(forward=phi, lateral=0, yaw=0)

    # 7. Print for debugging
    print(f"Distance: {actual_distance:.3f} m | Error: {error:.3f} | Phi: {phi:.3f}")
