

class IRSensor:
    def __init__(self, sensor, polling_rate):
        """
            Initialize the IR sensor and create the linear interpolation values for converting
            measured values into a distance in m. Values taken from sensor datasheet.
        """
        self.sensor = sensor
        self.sensor.enable(polling_rate)

        lookup_table_vals = [654, 614, 552, 491, 327, 276, 225, 184, 153, 123, 82]
        lookup_table_dists = [0.06, 0.07, 0.08, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.8]

        self.pairs = list(zip(
            zip(lookup_table_vals, lookup_table_vals[1:]),
            zip(lookup_table_dists, lookup_table_dists[1:])
        ))

    def get_distance(self):
        """
            Returns distance in m by performing linear interpolation of measured IR sensor value.
            If object is closer than 0.06m or further than 0.8m, returns float('inf') instead,
            as value is out of range of sensor capability.
        """
        x = self.sensor.getValue()
        y = float('inf')
        if x is None:
            print('[Warning]: Attempted IR sensor reading when no data available')
            return y 
        if x > 654:  # maximum received value from sensor, anything more is too close
            # print("attempted to calculate distance for value closer than sensor capability")
            return y
        for val_pair in self.pairs:
            (r1, r2), (d1, d2) = val_pair
            if r2 <= x < r1:
                y = ((x - r1) / (r2 - r1) * (d2 - d1)) + d1
        return y + 0.006 # changed due to a change in noise