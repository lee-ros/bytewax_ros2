import time
from typing import Callable

from .threshold import Threshold
from .threshold_direction import ThresholdDirection


class TimeAndValueThreshold(Threshold):
    def __init__(
        self,
        value_threshold: float,
        time_threshold: float,
        callback: Callable[[float], None],
        direction=ThresholdDirection.ABOVE,
    ):
        super().__init__(value_threshold, callback, direction)

        self._time_threshold = time_threshold
        self._crossed_stamped = None

    def clear(self):
        super().clear()
        self._crossed_stamped = None

    def execute(self, value):
        if self._time_passed():
            super().execute(value)

    def _time_passed(self):
        if self._crossed_stamped is None:
            self._crossed_stamped = time.monotonic()

        return time.monotonic() - self._crossed_stamped > self._time_threshold
