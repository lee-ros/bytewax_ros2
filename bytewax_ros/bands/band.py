from bytewax_ros.bands import BandSeverity


class Band:
    def __init__(
        self,
        lower_limit: float,
        upper_limit: float,
        severity: BandSeverity,
    ):
        if lower_limit > upper_limit:
            raise ValueError("Lower limit can't be bigger than upper limit")
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.severity = severity

    def check_sample(self, sameple: float) -> bool:
        return self._is_sample_in_band(sameple)

    def _is_sample_in_band(self, sample: float) -> bool:
        return self.lower_limit <= sample <= self.upper_limit

    def intersect(self, other: "Band"):
        return not (
            self.upper_limit <= other.lower_limit
            or self.lower_limit >= other.upper_limit
        )

    def __repr__(self):
        return f"band from: {self.lower_limit} to: {self.upper_limit}"
