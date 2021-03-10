import math


class ServoArmController:
    def __init__(self):
        self.scan_started = False
        self.clockwise = True
        self.scanning_speed = 0.01

    def stationary_scan(self, positioning_system):
        if self.scan_started:
            if positioning_system.get_turret_angle() == math.pi:
                print("stationary scan complete")
                self.scan_started = False
                return True
            positioning_system.spin_turret(self.scanning_speed)
        elif positioning_system.get_turret_angle() > 0.00:
            positioning_system.spin_turret(-self.scanning_speed)
        else:
            print("running stationary scan")
            self.scan_started = True
        return False

    def driving_scan(self, positioning_system):
        if self.clockwise:
            positioning_system.spin_turret(self.scanning_speed)
        else:
            positioning_system.spin_turret(-self.scanning_speed)

        if positioning_system.get_turret_angle() > 3 / 4 * math.pi:
            self.clockwise = False
        elif positioning_system.get_turret_angle() < 1 / 4 * math.pi:
            self.clockwise = True
