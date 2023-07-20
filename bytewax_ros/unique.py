from typing import Any


class UniqueFilter:
    """This class is used to filter duplicate items in a stream of samples"""

    def __init__(self):
        self._last_unique_sample = None

    def __call__(self, sample: Any) -> bool:
        if self._last_unique_sample == sample:
            return False

        self._last_unique_sample = sample
        return True
