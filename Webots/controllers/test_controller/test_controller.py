"""test_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math


def get_bearing(angle):
    rad = math.atan2(angle[0], angle[2])
    bearing = rad - 1.5708
    return bearing

if __name__ == "__main__":
    robot = Robot()
    timestep = 64
    max_speed = 6.28
    pi = 3.141592
    
    # left motor = motor_1
    # right motor = motor_2
    # for both motors, +ve speed = forward
    left_motor = robot.getDevice("drive_motor_1")
    right_motor = robot.getDevice("drive_motor_2")
    turret_motor = robot.getDevice("turret_motor")
    left_pincer = robot.getDevice("pincer_motor_1")
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    turret_motor.setPosition(float('inf'))
    turret_motor.setVelocity(0.0)
    
    left_pincer.setPosition(0.0)
    left_pincer.setVelocity(0.0)
    
    gps.enable(50)
    compass.enable(50)
    
    waypoint = (1, -1)
    p = 8.0
    
    while robot.step(timestep) != -1:  
        turret_motor.setVelocity(max_speed/2)
        
        left_pincer.setPosition(pi/4.0)
        left_pincer.setVelocity(pi/4.0)
        
        #both lists of floats
        angle = compass.getValues()
        position = gps.getValues()
        
        bearing = get_bearing(angle) + pi/2
        goal_bearing = math.atan2(waypoint[1] - position[2], waypoint[0] - position[0]) 
        error = bearing - goal_bearing
        print(bearing)
        print(goal_bearing)
        print(" ")
        #print(error)
        print(position)
        left_speed = min(max(max_speed/2 - error * p, -max_speed), max_speed)
        right_speed = min(max(max_speed/2 + error * p, -max_speed), max_speed)
        
        
        
        if abs(position[0] - 1) < 0.1 and abs(position[2] - 1) < 0.1:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
        else:
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
        
        
        
        
        
        
        
        
        
       
