

class IRSensor:
    def __init__(self, sensor, polling_rate):
        self.sensor = sensor
        self.sensor.enable(polling_rate)

        lookup_table_vals = [654, 614, 552, 491, 327, 276, 225, 184, 153, 123, 82]
        lookup_table_dists = [0.06, 0.07, 0.08, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.8]

        lookup_table_dists_pairs = [[lookup_table_dists[i], lookup_table_dists[i + 1]]
                                    for i in range(len(lookup_table_dists) - 1)]
        self.lookup_table_vals_pairs = [[lookup_table_vals[i], lookup_table_vals[i + 1]]
                                        for i in range(len(lookup_table_vals) - 1)]

        self.pairs = []

        for m, n in zip(self.lookup_table_vals_pairs, lookup_table_dists_pairs):
            self.pairs.append([m, n])

    def get_distance(self):
        x = self.sensor.getValue()
        y = float('inf')
        for index, val_pair in enumerate(self.lookup_table_vals_pairs):
            if (x < val_pair[0] and x >= val_pair[1]):
                y = ((x - self.pairs[index][0][0]) / (self.pairs[index][0][1] - self.pairs[index][0][0]) *
                     (self.pairs[index][1][1] - self.pairs[index][1][0])) + self.pairs[index][1][0]
        return y
