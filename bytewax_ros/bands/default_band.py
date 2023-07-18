import math

from bytewax_ros.bands import Band, BandSeverity


class DefaultBand(Band):
    def __init__(self, severity: BandSeverity):
        super().__init__(-math.inf, math.inf, severity)
