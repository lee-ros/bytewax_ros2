from typing import Any, Optional, TypeVar


T = TypeVar('T')


class Unique:
    def __init__(self):
        self._last_unique_sample = None

    def __call__(self, sample: T) -> Optional[T]:
        if self._last_unique_sample != sample:
            self._last_unique_sample = sample
            return sample
