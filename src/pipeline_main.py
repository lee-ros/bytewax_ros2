from functools import partial
import threading
from time import sleep
from typing import Any

import rclpy

from bytewax.dataflow import Dataflow
from std_msgs import msg as std_msg
from rclpy.node import Node
from rclpy.utilities import try_shutdown as rclpy_shutdown

from bytewax_ros.connectors import RosTopicInput, RosTopicOutput
from bytewax_ros.thresholds import Threshold, ThresholdHandler
from bytewax_ros.utils import run_flow_as_thread, value_from_obj


def build_message(data: float) -> std_msg.String:
    string = f"Thread-{threading.current_thread().name}: {data}"
    return std_msg.String(data=string)


def execute_threshold(threshold: Threshold) -> Any:
    return threshold.execute_once()


def create_flow(node: Node):
    inp_msg_type = std_msg.Float32
    inp_topic_name = "/data_in"
    out_msg_type = std_msg.String
    out_topic_name = "/data_out"

    threshold_handler = ThresholdHandler(
        [
            Threshold(threshold=1, callback=partial(build_message, 1.0)),
            Threshold(threshold=2, callback=partial(build_message, 2.0)),
            Threshold(threshold=3, callback=partial(build_message, 3.0)),
        ]
    )

    flow = Dataflow()
    flow.input("inp", RosTopicInput(node, inp_msg_type, inp_topic_name))
    flow.map(lambda msg: value_from_obj(msg, "obj.data"))
    flow.filter_map(threshold_handler.select_threshold)
    flow.filter_map(execute_threshold)
    flow.output("out", RosTopicOutput(node, out_msg_type, out_topic_name))

    return flow


def main():
    rclpy.init()
    node = Node("flow_node")  # type: ignore

    run_flow_as_thread("flow1", create_flow(node))
    run_flow_as_thread("flow2", create_flow(node))

    while True:
        try:
            rclpy.spin_once(node)
            sleep(0)
        except KeyboardInterrupt:
            break

    rclpy_shutdown()


if __name__ == "__main__":
    main()
