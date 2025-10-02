
#Task1 - less distance slow down 

# e(t) = a-d
# Desired (d) is distance to the wall 
# actual (a) is the actual distance, gotten from the lidar. degrees 

# For lidar (a): do not take the average of the distance, Minimums will be best to use 
# [choose->123, 153] of these range do not take average 


# Phi(t) = k(p)*(e(t))
# Where phi is angular velocity and k(p) is a choosen variable k(p) = 3 is a good choice 



#Task2 - Wall folllowing 
#USE PID
#Use range of values from the lidar to follow the wall.
#we need a variable that will determine how bigof a range we want from the range.
#Remember to always use the minimum, the minimum will let you know how close the bot is to the wall 


# e(s)(t) = |a-d|
#(change in)P(t) = K(Ps) * e(s)
#phi(right)(t) = 

#if a < d
    #
#if a > b
def sat(self, v):
    if v> self.max_motor_velocity:
        return self.max_motor_velocity
    elif v< -self.max_motor_velocity:
        return -self.max_motor_velocity
    else:
        return v


#forward distance  and kp can be changed 
def forward_PID(self, forward_distance = 0.5, Kp = 3):   #can be used for task 1 and 2 if we change kp then the bot wil start slowing down accordingly 
    actual = min(self.get_lidar_range_image()[175:185])
    e= actual - forward_distance
    forward_v = self.sat(Kp * e)
    return forward_v


def side_PID(self, side_distance=0.1, Kp=3, side = 'Left'):  #side_distance=0.1 <- these values can be an issue
    if side == 'Left':
        actual = min(self.get_lidar_range_image()[90:115])
        e= abs(actual - side_distance)
        return Kp * e
    
def Rotation (sef,degrees):
    
#bot.move_to_start()
def main ():  
    while bot.experiment_supervisor.step(bot.timestamp) !=1:
        forward_distance =bot.get_front_distance()
        if forward_distance < bot.min_forward_wall_distance :
            bot.rotate (-45)
        forward_velocity =bot.forward_PID()
        right_v = forward_velocity
        left_v = forward_velocity
        if side == "left":
            delta_velocity = bot.side_PID(side="left")
            side_distance = jeff.get_left_side_distance()

            #too close
            if side_distance < bot.desired_wall_follow_distance:
                left_v = bot.sat(left_v+delta_velocity)        
                right_v = bot.sat(ight_v+delta_velocity)     
            #Too Far 
            elif side_distance > bot.desired_wall_follow_distance:
                right_v = bot.sat(left_v+delta_velocity)        
                left_v = bot.sat(ight_v+delta_velocity)
        else:
            #right wall following
            pass
    # forward_distance = min(bot.get_lidar_range_image()[175:185])
    # forward_velocity =  bot.forward_PID()
    # delta_velocity = bot.side_PID()
    # side_distance =min(bot.get_lidar_range_image()[90:115])
    
    # if side_distance < .2: #bot to close to wall
    #     left_v = forward_velocity
    #     right_v = bot.sat(forward_velocity -delta_velocity)
    
    # elif(side_distance > .2):
    #     left_v = bot.sat(forward_velocity -delta_velocity)
    #     right_v = forward_velocity
    # else:
    #     left_v = forward_velocity
    #     right_v = forward_velocity

bot.set_left_motor_velocity(left_v)
bot.set_right_motor_velocity(right_v)

