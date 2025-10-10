from robot_systems.robot import HamBot
import time

bot = HamBot(lidar_enabled=True, camera_enabled=False)
while True:
    scan = bot.get_range_image()
    if scan == -1:
        print("No data")
        time.sleep(0.5)
        continue

    # print a few beams around what you think is 'front'
    for i in [170,175,180,185,190]:
        print(f"{i:3d}: {scan[i]:.2f}", end=" | ")
    print()
    time.sleep(0.3)
