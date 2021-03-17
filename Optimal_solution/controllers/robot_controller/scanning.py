import math


class ServoArmController:
    def __init__(self):
        self.scan_started = False
        self.clockwise = True
        self.scanning_speed = 0.01

    def stationary_scan(self, positioning_system):
        if self.scan_started:
            # This is very bad code. I wouldn't have approved the merge if I saw it
            if positioning_system.get_turret_angle() == math.pi:
                self.scan_started = False
                return True
            positioning_system.spin_turret(self.scanning_speed)
        elif positioning_system.get_turret_angle() > 0.00:
            positioning_system.spin_turret(-self.scanning_speed)
        else:
            self.scan_started = True
        return False

    def active_scan(self, positioning_system):
        # Move turret
        if self.clockwise:
            positioning_system.spin_turret(self.scanning_speed)
        else:
            positioning_system.spin_turret(-self.scanning_speed)

        # Should the scan continue
        if positioning_system.get_turret_angle() > 7 / 8 * math.pi:
            self.clockwise = False
        elif positioning_system.get_turret_angle() < 2 / 8 * math.pi:
            self.clockwise = True
            return True

        return False

    def reset_angle(self, positioning_system):
        positioning_system.spin_turret(-self.scanning_speed)
        return positioning_system.get_turret_angle() < 1 / 4 * math.pi

    def driving_scan(self, positioning_system):
        if self.clockwise:
            positioning_system.spin_turret(self.scanning_speed)
        else:
            positioning_system.spin_turret(-self.scanning_speed)

        if positioning_system.get_turret_angle() > 3 / 4 * math.pi:
            self.clockwise = False
        elif positioning_system.get_turret_angle() < 1 / 4 * math.pi:
            self.clockwise = True
