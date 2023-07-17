import operator
from typing import Any, Callable

from .threshold_direction import ThresholdDirection


class Threshold:
    def __init__(
        self,
        threshold: float,
        callback: Callable[[], Any],
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

    def check_threshold(self, value: float) -> bool:
        crossed = self._comparison_op(value, self._threshold)

        if not crossed:
            self.clear()

        return crossed

    def check_threshold_with_callback(self, value: float) -> bool:
        crossed = self.check_threshold(value)

        if crossed and not self._called:
            self.execute()

        return crossed

    def clear(self):
        self._called = False

    def execute(self) -> Any:
        callback_return_value = None

        if self._callback is not None:
            callback_return_value = self._callback()
            self._called = True

        return callback_return_value

    def execute_once(self) -> Any:
        if not self._called:
            return self.execute()

    def _select_operand_by_direction(
        self, direction: ThresholdDirection
    ) -> Callable:
        if direction == ThresholdDirection.ABOVE:
            return operator.gt
        return operator.lt