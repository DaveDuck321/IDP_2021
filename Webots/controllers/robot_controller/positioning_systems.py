from common import util
import math

SPEED_OF_SOUND = 343


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
        self._turret_motor = turret_motor
        self._turret_motor.setPosition(math.inf)
        self._turret_motor.setVelocity(0.0)

        self.__turret_angle = 0

    def set_turret_angle(self, angle):
        """
            Moves the turret motor to the specified angle.
        """
        self.turret_motor.setPosition(angle)
        self.__turret_angle = angle

    def get_turret_angle(self, angle):
        """
            Get the current turret angle.
            note: set_turret_angle should not be called shortly before
        """
        return self.__turret_angle

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

    def get_distance_readings(self):
        """
            Returns transformed distances as measured by the distance sensors.
            (front_read, rear_reading)
        """
        return (
            get_echo_distance(self._front_distance.getValue()),
            get_echo_distance(self._rear_distance.getValue())
        )
