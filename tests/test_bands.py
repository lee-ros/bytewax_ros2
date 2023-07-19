import pytest

from bytewax_ros.bands import Band, BandSeverity, SampleTriggeredBand


@pytest.mark.parametrize(
    "band1, band2, expected", 
    [
        (Band(0, 5, BandSeverity.OK), Band(1, 3, BandSeverity.OK), True),
        (Band(0, 1, BandSeverity.OK), Band(1, 3, BandSeverity.OK), False),
        (Band(0, 5, BandSeverity.OK), Band(4, 6, BandSeverity.OK), True),
        (Band(3, 5, BandSeverity.OK), Band(1, 4, BandSeverity.OK), True),
        (Band(3, 5, BandSeverity.OK), Band(1, 3, BandSeverity.OK), False),
    ]
)
def test_band_intersection(band1: Band, band2: Band, expected: bool):
    assert band1.intersect(band2) == expected


def test_band_check_sample():
    band = Band(0, 5, BandSeverity.OK)
    sample_true = 1
    sample_false = 6
    
    assert band.check_sample(sample_true)
    assert not band.check_sample(sample_false)


def test_sample_triggerd_band_check_sample():
    num_of_samples = 2
    sample_true = 4
    band = SampleTriggeredBand(0, 5, BandSeverity.OK, num_of_samples)
    
    for i in range(num_of_samples):
        if i < num_of_samples - 1:
            assert not band.check_sample(sample_true)
        else:
            assert band.check_sample(sample_true)
