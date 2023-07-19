from bytewax_ros.bands import Band, BandSeverity


class SampleTriggeredBand(Band):
    def __init__(
        self,
        lower_limit: float,
        upper_limit: float,
        severity: BandSeverity,
        number_of_samples: int = 1,
    ):
        super().__init__(lower_limit, upper_limit, severity)

        if number_of_samples < 1:
            raise ValueError("Number of samples must be a positive integer")

        self._number_of_samples = number_of_samples
        self._consecutive_samples_in_band = 0

    def check_sample(self, sample: float) -> bool:
        if not self._is_sample_in_band(sample):
            return False
        if self._consecutive_samples_in_band < self._number_of_samples:
            return False
        return True

    def _is_sample_in_band(self, sample: float) -> bool:
        result = super()._is_sample_in_band(sample)
        self._consecutive_samples_in_band = (
            self._consecutive_samples_in_band + 1 if result else 0
        )
        return result
