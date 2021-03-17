from common import util
import math
import IR_block_search

ARM_RADIUS = 0.15  # 150 mm
SPEED_OF_SOUND = 343
PINCER_OFFSET = 0.1


def get_echo_distance(time_us):
    """
        Returns the distance (d) between a sound source and the reflection point.
        The echo pulse was received time_us after the initial pulse and traveled 2*d.
    """
    return (SPEED_OF_SOUND * time_us) / (2e6)


class PositioningSystem:
    def __init__(self, turret_motor, gps, compass,
                 front_distance, rear_distance, sampling_rate=5):
        self._gps = gps
        self._compass = compass
        self._front_distance = front_distance
        self._rear_distance = rear_distance

        # Initialize sensors
        self._gps.enable(sampling_rate)
        self._compass.enable(sampling_rate)
        self._front_distance.enable(sampling_rate)
        self._rear_distance.enable(sampling_rate)

        # Motor for sensor turret
        self.__turret_target_angle = 0
        self.__target_velocity = turret_motor.getMaxVelocity()

        self._turret_motor = turret_motor
        self._turret_motor.setPosition(0)
        self._turret_motor.setVelocity(self.__target_velocity)

    def kill_turret(self):
        self._turret_motor.setVelocity(0)

    def spin_turret(self, speed):
        self.set_turret_angle(self.get_turret_angle() + speed)

    def set_turret_angle(self, angle):
        """
            Moves the turret motor to the specified angle.
        """
        angle = min(max(0, angle), math.pi)
        self._turret_motor.setPosition(angle)
        self.__turret_target_angle = angle

    def get_turret_angle(self):
        """
            Get the current turret angle.
            note: set_turret_angle should not be called shortly before
        """
        return self.__turret_target_angle

    def get_2D_position(self):
        """
            Returns the current transformed world position of the GPS.
        """
        gps_x, _, gps_z = self._gps.getValues()
        return (gps_x, gps_z)

    def get_world_bearing(self, sign=1):
        """
            Returns the world bearing of the compass in radians.
        """
        compass_data = self._compass.getValues()
        return util.get_bearing((sign * compass_data[2], sign * compass_data[0]), (0, 0))

    def get_bearing_error(self, target, reverse=False):
        """
            Returns the corrected bearing error between current bearing and the bearing
            of a target point.
        """
        current_position = self.get_2D_position()
        current_bearing = self.get_world_bearing()
        if reverse:
            current_bearing = (current_bearing + math.pi) % (2 * math.pi)

        goal_bearing = util.get_bearing(target, current_position)

        # correct for errors due to current bearing and target bearing being between 0 to 2pi
        bearing_error = current_bearing - goal_bearing
        if bearing_error > math.pi:
            bearing_error -= 2 * math.pi
        elif bearing_error < -math.pi:
            bearing_error += 2 * math.pi
        return bearing_error

    def get_distance_readings(self):
        """
            Returns transformed distances as measured by the distance sensors.
            (front_read, rear_reading)
        """
        return (
            get_echo_distance(self._front_distance.getValue()),
            get_echo_distance(self._rear_distance.getValue())
        )

    def block_detection_position(self):
        """
            Returns the furthest position at which the robot should be able to detect a block.
        """

        robot_pos = self.get_2D_position()
        robot_bearing = self.get_world_bearing()
        return (
            robot_pos[0] + IR_block_search.MAX_BLOCK_DIST * math.cos(robot_bearing),
            robot_pos[1] + IR_block_search.MAX_BLOCK_DIST * math.sin(robot_bearing)
        )

    def get_pincer_position(self):
        robot_pos = self.get_2D_position()
        robot_bearing = self.get_world_bearing()
        return (
            robot_pos[0] + PINCER_OFFSET * math.cos(robot_bearing),
            robot_pos[1] + PINCER_OFFSET * math.sin(robot_bearing)
        )
