from typing import List, Optional

from bytewax_ros.bytewax_ros.thresholds import DefaultThreshold, Threshold, ThresholdDirection


class ThresholdHandler:
    def __init__(self, thresholds: List[Threshold]):
        super().__init__()
        self._thresholds = self._sort_thresholds(thresholds)

    def _sort_thresholds(self, thresholds: List[Threshold]) -> List[Threshold]:
        default_thresh = self._get_default_threshold(thresholds)
        above_threshs = self._get_sorted_thresholds_by_direction(
            thresholds, ThresholdDirection.ABOVE
        )
        below_threshs = self._get_sorted_thresholds_by_direction(
            thresholds, ThresholdDirection.BELOW
        )

        # the order of the thesholds is critical!
        return above_threshs + below_threshs + default_thresh

    def _get_default_threshold(self, thresholds: List[Threshold]) -> List[Threshold]:
        default_thresh = [
            thresh for thresh in thresholds if isinstance(thresh, DefaultThreshold)
        ]

        if len(default_thresh) == 0:
            return []

        elif len(default_thresh) == 1:
            thresholds.remove(default_thresh[0])
            return default_thresh

        raise ValueError("Only one default threshold is allowed")

    def _get_sorted_thresholds_by_direction(
        self, thresholds: List[Threshold], direction: ThresholdDirection
    ) -> List[Threshold]:
        reverse = direction == ThresholdDirection.ABOVE

        threshs = [thresh for thresh in thresholds if thresh.direction == direction]
        threshs.sort(key=lambda thresh: thresh.threshold, reverse=reverse)

        return threshs

    def select_threshold(self, value) -> Optional[Threshold]:
        for threshold in self._thresholds:
            crossed = threshold.check_threshold(value)

            if crossed:
                self._clear_not_crossed_thresholds(crossed_threshold=threshold)
                return threshold, value

    def _clear_not_crossed_thresholds(self, *, crossed_threshold: Threshold = None):
        for t in self._thresholds:
            if t is not crossed_threshold:
                t.clear()
