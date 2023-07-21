from typing import List, Optional

from bytewax_ros.bands import Band, DefaultBand
from bytewax_ros.bands import utilities as band_utils


class BandDetector:
    def __init__(
        self,
        bands: List[Band],
        *,
        default_band: Optional[DefaultBand] = None,
        verify_no_gaps=True
    ):
        band_utils.verify_no_intersections(bands)
        
        if verify_no_gaps:
            band_utils.verify_no_gaps(bands)
        
        self._bands = bands
        self._default_band = default_band

    def detect_band(self, sample: float) -> Optional[Band]:
        bands = [band for band in self._bands if band.check_sample(sample)]

        if bands:
            return bands[0]
        if self._default_band is not None:
            return self._default_band
        return None
