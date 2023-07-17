from functools import partial
from threading import Thread
from typing import Any

from bytewax.dataflow import Dataflow
from bytewax.run import cli_main


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


def run_flow_as_thread(name: str, flow: Dataflow) -> Thread:
    """Run a `DataFlow` object in a thread.
    
    This function is used for running a flow object in parallel to the ros node spinning
    
    Args:
        name: the name of the thread
        flow: the `DataFlow` to run
        
    Returns:
        the thread instance in which the flow runs
    """
    flow_args = {
        "flow": flow,
        "recovery_config": None,
    }

    t = Thread(name=name, target=partial(cli_main, **flow_args))
    t.start()

    return t
