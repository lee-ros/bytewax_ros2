from functools import partial
from threading import Thread
from typing import Any

from bytewax.dataflow import Dataflow
from bytewax.run import cli_main


def value_from_obj(obj: Any, attribute: str) -> Any:
    return eval(attribute, {}, {"obj": obj})


def run_flow_as_thread(name: str, flow: Dataflow) -> Thread:
    flow_args = {
        "flow": flow,
        "recovery_config": None,
    }

    t = Thread(name=name, target=partial(cli_main, **flow_args))
    t.start()

    return t
