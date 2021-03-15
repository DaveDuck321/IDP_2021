FLUFFY_THRESHOLD = 500


class ColorSensor:
    def __init__(self, light_sensor, sampling_rate):
        self._sensor = light_sensor
        self._sensor.enable(sampling_rate)

    def scan_color(self):
        """
            Returns the color of the scanned block: either "Small" or "Fluffy".
        """
        raw_result = self._sensor.getValue()

        if raw_result > FLUFFY_THRESHOLD:
            return "Fluffy"

        return "Small"
