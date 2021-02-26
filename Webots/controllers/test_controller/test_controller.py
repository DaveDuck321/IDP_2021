"""test_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math, time


class MobileRobot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = 25
        
        # motors
        self.left_drive_motor = self.robot.getDevice("drive_motor_1")
        self.right_drive_motor = self.robot.getDevice("drive_motor_2")
        self.turret_motor = self.robot.getDevice("turret_motor")
        self.left_pincer = self.robot.getDevice("pincer_motor_1")
        self.right_pincer = self.robot.getDevice("pincer_motor_2")
        self.initialize_motors()
        
        # sensors
        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.front_distance = self.robot.getDevice("distance_sensor_front")
        # self.rear_distance = self.robot.getDevice("distance_sensor_rear")
        self.initialize_sensors()
        
        self.open_pincer()
        
        # test drive_toward_point()
        self.stationary_scan()
        #self.navigate_between_waypoints()
        
        
    def initialize_motors(self):
        self.left_drive_motor.setPosition(float('inf'))
        self.left_drive_motor.setVelocity(0.0)
        
        self.right_drive_motor.setPosition(float('inf'))
        self.right_drive_motor.setVelocity(0.0)
        
        self.turret_motor.setPosition(float('inf'))
        self.turret_motor.setVelocity(0.0)
        
        max_servo_velocity = self.left_pincer.getMaxVelocity()
        self.left_pincer.setPosition(0.0)
        self.left_pincer.setVelocity(max_servo_velocity)
        
        self.right_pincer.setPosition(0.0)
        self.right_pincer.setVelocity(max_servo_velocity)
        
         
    def initialize_sensors(self):
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.front_distance.enable(self.timestep)
        # self.rear_distance.enable(25)
        
        
    def open_pincer(self):
        open_angle = 3.141592 / 4.0
        
        self.left_pincer.setPosition(open_angle)
        self.right_pincer.setPosition(-open_angle)
        
        
    def close_pincer(self):      
        self.left_pincer.setPosition(0.0)
        self.right_pincer.setPosition(0.0)
        
    def distance_data(self, angle):
        front_dist = self.front_distance.getValue()
        front_angle = angle
        # rear_dist = self.rear_distance.getValues()
        # rear_angle = angle + 3.141592
        # trasmit data
        print(front_dist, front_angle)
        
    def stationary_scan(self):
        print("start")
        self.stop_driving()
        self.turret_motor.setPosition(0.0)
        self.turret_motor.setVelocity(self.turret_motor.getMaxVelocity())
        NUM_SCANS = 100
        
        while self.robot.step(self.timestep) != -1:
            for i in range(NUM_SCANS):
                self.turret_motor.setPosition(3.141592 * i/NUM_SCANS)
                self.robot.step(self.timestep)
                self.distance_data(3.141592 * i/NUM_SCANS)
            print("finished scan")
            return 
            
        
        
    
    def stop_driving(self):
        self.left_drive_motor.setVelocity(0.0)
        self.right_drive_motor.setVelocity(0.0)
         
        
    def drive_toward_point(self, destination):
        # read value of maximum motor speed from object itself
        max_speed = self.left_drive_motor.getMaxVelocity()
        
        # the importance of this value depends on the distance away the point is
        # some fine tuning later down the line may be necessary
        p = 10.0
        
        gps_data = self.gps.getValues()
        compass_data = self.compass.getValues()
        
        # Use proportional control based on error in orientation between
        # current robot orientation and orientation required to drive toward point
        bearing = math.atan2(compass_data[0], compass_data[2])
        goal_bearing = math.atan2(destination[1] - gps_data[2], destination[0] - gps_data[0])
        
        # this fixes the issue when target bearing is close to 3.1... If it goes even slightly over, it flips to -3.1...
        # causing the robot to think it needs to turn the other way (does a strange spinning shuffle, very bad)
        # there is still an issue when target bearing is exactly opposite current bearing, however seems to resolve itself
        # harmlessly. Needs investigating.
        if bearing > 3.141592 / 2.0 or bearing < -3.141592 / 2.0:
            bearing = math.atan2(-compass_data[0], -compass_data[2])
            goal_bearing = math.atan2(-destination[1] + gps_data[2], -destination[0] + gps_data[0]) 
           
        error = bearing - goal_bearing
        print(bearing)
        print(goal_bearing)
        # make sure drive motor speeds do not exceed maximum speed in forward or reverse
        left_speed = min(max(max_speed/2 - error * p, -max_speed), max_speed)
        right_speed = min(max(max_speed/2 + error * p, -max_speed), max_speed)
        
        # set drive motor speed
        self.left_drive_motor.setVelocity(left_speed)
        self.right_drive_motor.setVelocity(right_speed)
        
        
    def navigate_between_waypoints(self):
        # this is merely a test function, to make sure waypoint navigation works
        # robot will move to each point in list sequencially, stopping still upon
        # reaching the final point
        waypoint = [(0.45, 0.38), (1, -1), (1, 1), (-1, 1), (-1, -1), (-1, 1), (1, 1), (1, -1), (-1, -1), (0, 0)]
        index = 0
        
        while self.robot.step(self.timestep) != -1:  
            gps_data = self.gps.getValues()
            compass_data = self.compass.getValues()
            
            self.drive_toward_point(waypoint[index])
            
            if abs(gps_data[0] - waypoint[index][0]) < 0.03 and abs(gps_data[2] - waypoint[index][1]) < 0.03:
                index += 1
                self.close_pincer()
                if index == len(waypoint):
                    self.left_drive_motor.setVelocity(0)
                    self.right_drive_motor.setVelocity(0)
                    self.close_pincer()
                    return



def main():
    fluffy = MobileRobot()
    

if __name__ == "__main__":
    main()
        
        
        
        
        
       
