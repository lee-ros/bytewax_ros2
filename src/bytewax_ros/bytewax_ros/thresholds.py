from enum import Enum, auto
import math
import operator
import time
from typing import Any, Callable


class ThresholdDirection(Enum):
    ABOVE = auto()
    BELOW = auto()


class Threshold:
    def __init__(
        self,
        threshold: float,
        callback: Callable[[float], Any],
        direction=ThresholdDirection.ABOVE,
    ):
        self._threshold = threshold
        self._callback = callback
        self._called = False
        self._direction = direction
        self._comparison_op = self._select_operand_by_direction(direction)

    @property
    def direction(self):
        return self._direction

    @property
    def threshold(self):
        return self._threshold

    def check_threshold(self, value: float):
        crossed = self._comparison_op(value, self._threshold)

        if not crossed:
            self.clear()

        return crossed

    def check_threshold_with_callback(self, value: float) -> bool:
        crossed = self.check_threshold(value)

        if crossed and not self._called:
            self.execute(value)

        return crossed

    def clear(self):
        self._called = False

    def execute(self, value: float):
        callback_return_value = None

        if self._callback is not None:
            callback_return_value = self._callback(value)
            self._called = True

        return callback_return_value

    def _select_operand_by_direction(self, direction: ThresholdDirection):
        if direction == ThresholdDirection.ABOVE:
            return operator.gt
        return operator.lt


class DefaultThreshold(Threshold):
    def __init__(self, callback: Callable[[float], None]):
        super().__init__(-math.inf, callback)


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
