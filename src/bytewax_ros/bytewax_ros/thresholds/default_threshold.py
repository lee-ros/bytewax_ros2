import math

from typing import Callable

from .threshold import Threshold


class DefaultThreshold(Threshold):
    def __init__(self, callback: Callable[[float], None]):
        super().__init__(-math.inf, callback)