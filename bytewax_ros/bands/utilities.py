from typing import List, Tuple

from bytewax_ros.bands import Band, BandSeverity


def find_intersections(bands: List[Band]) -> List[Tuple[Band, Band]]:
    """Find intersections in a list of bands

    Args:
        bands: list of bands to search in

    Returns:
        A list of two-tuple containing the intersecting bands
    """
    intersections = []
    bands.sort(key=lambda band: band.lower_limit)

    for i in range(len(bands) - 1):
        if bands[i].intersect(bands[i + 1]):
            intersections.append((bands[i], bands[i + 1]))
    return intersections


def verify_no_intersections(bands: List[Band]):
    """Verify that a given list of bands has no intersections

    Args:
        bands: list of bands to verify

    Raises:
        `ValueError` if the bands has intersection
    """
    intersections = find_intersections(bands)
    if intersections:
        raise ValueError(f"Intersections found at: {intersections}")


def verify_no_gaps(bands: List[Band]):
    """Verify that a given list of bands has no gaps between bands

    Args:
        bands: list of bands to verify

    Raises:
        `ValueError` if the bands has gaps
    """
    bands.sort(key=lambda band: band.lower_limit)
    for i in range(len(bands) - 1):
        if bands[i].upper_limit != bands[i + 1].lower_limit:
            raise ValueError("Gaps found in the given list of bands")
