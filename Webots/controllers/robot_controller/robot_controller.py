"""robot_controller controller."""

from controller import Robot
import math
import time
import struct


class RobotController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = 5

        # Get all motors
        self.left_drive_motor = self.robot.getDevice("drive_motor_1")
        self.right_drive_motor = self.robot.getDevice("drive_motor_2")
        self.turret_motor = self.robot.getDevice("turret_motor")
        self.left_pincer = self.robot.getDevice("pincer_motor_1")
        self.right_pincer = self.robot.getDevice("pincer_motor_2")
        self.initialize_motors()

        # Get all sensors
        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.front_distance = self.robot.getDevice("distance_sensor_front")
        self.rear_distance = self.robot.getDevice("distance_sensor_rear")
        self.light = self.robot.getDevice("light_sensor")
        
        self.emitter = self.robot.getDevice(
            "emitter")    # channel 1 robot -> controller
        self.receiver = self.robot.getDevice(
            "receiver")  # channel 2 controller -> robot
        self.initialize_sensors()

        self.open_pincer()

        self.stationary_scan()
        #self.get_waypoints()
        self.navigate_between_waypoints([0.5, 0.45])
        self.color_data()

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
        self.receiver.enable(self.timestep)
        self.front_distance.enable(self.timestep)
        self.rear_distance.enable(self.timestep)
        self.light.enable(self.timestep)

    def open_pincer(self):
        open_angle = 3.141592 / 4.0

        self.left_pincer.setPosition(open_angle)
        self.right_pincer.setPosition(-open_angle)

    def close_pincer(self):
        self.left_pincer.setPosition(0.0)
        self.right_pincer.setPosition(0.0)

    def distance_data(self, angle, bearing, position):
        pos_x, pos_y, pos_z = position

        front_dist = self.front_distance.getValue()
        rear_dist = self.rear_distance.getValue()

        # check that no values are nan
        if math.isnan(front_dist):
            front_dist = float('inf')
        if math.isnan(rear_dist):
            rear_dist = float('inf')
        if math.isnan(angle):
            angle = float('inf')
        if math.isnan(bearing):
            bearing = float('inf')
        if math.isnan(pos_x):
            pos_x = float('inf')
        if math.isnan(pos_z):
            pos_z = float('inf')
        data = struct.pack("6f", front_dist, rear_dist,
                           angle, bearing, pos_x, pos_z)
        self.emitter.send(data)

    def color_data(self):
        data = self.light.getValue()
        print(data)

    def stationary_scan(self):
        self.stop_driving()
        self.robot.step(self.timestep)
        gps_data = self.gps.getValues()
        compass_data = self.compass.getValues()
        bearing = math.atan2(compass_data[0], compass_data[2])

        self.turret_motor.setPosition(0.0)
        self.turret_motor.setVelocity(self.turret_motor.getMaxVelocity())
        NUM_SCANS = 150

        while self.robot.step(self.timestep) != -1:
            for i in range(NUM_SCANS):
                self.turret_motor.setPosition(3.141592 * i/NUM_SCANS)
                self.robot.step(self.timestep)
                self.distance_data(3.141592 * i/NUM_SCANS, bearing, gps_data)
            return

    def stop_driving(self):
        self.left_drive_motor.setVelocity(0.0)
        self.right_drive_motor.setVelocity(0.0)

    def get_waypoints(self):
        num_waypoints = -1
        while self.robot.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                data = self.receiver.getData()
                if num_waypoints == -1:
                    num_waypoints = struct.unpack("f", data)[0]
                elif num_waypoints > 0:
                    print('{}f'.format(num_waypoints))
                    waypoints = struct.unpack(
                        '{}f'.format(int(num_waypoints)), data)
                    self.navigate_between_waypoints(waypoints)

                self.receiver.nextPacket()

    def drive_toward_point(self, destination):
        max_speed = self.left_drive_motor.getMaxVelocity()

        # the importance of this value depends on the distance away the point is
        # some fine tuning later down the line may be necessary
        p = 20.0

        gps_data = self.gps.getValues()
        compass_data = self.compass.getValues()

        # Use proportional control based on error in orientation between
        # current robot orientation and orientation required to drive toward point
        bearing = math.atan2(compass_data[0], compass_data[2])
        goal_bearing = math.atan2(
            destination[1] - gps_data[2], destination[0] - gps_data[0])

        if bearing > 3.141592 / 2.0 or bearing < -3.141592 / 2.0:
            bearing = math.atan2(-compass_data[0], -compass_data[2])
            goal_bearing = math.atan2(-destination[1] +
                                      gps_data[2], -destination[0] + gps_data[0])

        error = bearing - goal_bearing

        if error > 0.0:
            left_speed = max(max_speed - error*p, -max_speed)
            right_speed = max_speed
        else:
            left_speed = max_speed
            right_speed = max(max_speed + error*p, -max_speed)

        # set drive motor speed
        self.left_drive_motor.setVelocity(left_speed)
        self.right_drive_motor.setVelocity(right_speed)

    def navigate_between_waypoints(self, waypoints):
        index = 0
        while self.robot.step(self.timestep) != -1:
            gps_data = self.gps.getValues()
            compass_data = self.compass.getValues()

            self.drive_toward_point((waypoints[index], waypoints[index+1]))

            if abs(gps_data[0] - waypoints[index]) < 0.03 and abs(gps_data[2] - waypoints[index+1]) < 0.03:
                index += 2
                self.close_pincer()
                if index == len(waypoints):
                    self.stop_driving()
                    self.close_pincer()
                    return


def main():
    fluffy = RobotController()


if __name__ == "__main__":
    main()
