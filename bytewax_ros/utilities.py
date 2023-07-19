from functools import partial
from threading import Thread
from typing import Any, List, Optional, Tuple

from bytewax.dataflow import Dataflow
from bytewax.run import cli_main

from bytewax_ros.bands import Band


def value_from_obj(obj: Any, attribute: str) -> Any:
    """Safely evaluates the value of an attribute path from an object

    Args:
        obj: the object to get the value from
        attribute: a string representing the path to the value, must begin with `obj` as the object name

    Returns:
        the evaluated value from the object

    Example:
        ```
        obj = {'a' : 5}
        value_from_obj(obj, "obj['a']")  # -> 5
        ```
    """
    return eval(attribute, {}, {"obj": obj})


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


def verify_no_intersections(bands: List[Band]) -> bool:
    """Verify that a given list of bands has no intersections

    Args:
        bands: list of bands to verify

    Returns:
        True if the list of bands has no intersections

    Raises:
        `ValueError` if the bands has intersection
    """
    intersections = find_intersections(bands)
    if intersections:
        raise ValueError(f"Intersections found at: {intersections}")
    return True


def run_flow(flow: Dataflow):
    flow_args = {
        "flow": flow,
        "recovery_config": None,
    }

    cli_main(**flow_args)


def run_flow_as_thread(name: str, flow: Dataflow) -> Thread:
    """Run a `DataFlow` object in a thread.

    This function is used for running a flow object in parallel to the ros node spinning

    Args:
        name: the name of the thread
        flow: the `DataFlow` to run

    Returns:
        the thread instance in which the flow runs
    """
    t = Thread(name=name, target=partial(run_flow, flow))
    t.start()

    return t
