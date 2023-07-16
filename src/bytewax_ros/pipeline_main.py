import logging

from functools import partial
from threading import Thread
from typing import Any, Tuple

import rclpy

from bytewax.dataflow import Dataflow
from bytewax.run import cli_main
from std_msgs import msg as std_msg
from rclpy.node import Node

from connectors import RosTopicInput, RosTopicOutput
from thresholds import Threshold
from threshold_handler import ThresholdHandler


def message_to_value(message: std_msg.Float32) -> float:
    return message.data


def log_value(value: float) -> float:
    logging.info("value=")
    return value


def execute_threshold(data: Tuple[Threshold, float]) -> Any:
    threshold, value = data
    return threshold.execute(value * threshold.threshold)


def threshold_result_to_message(result: float) -> std_msg.Float32:
    return std_msg.Float32(data=result)


def main():
    threshold_handler = ThresholdHandler([
        Threshold(threshold=1, callback=log_value),
        Threshold(threshold=2, callback=log_value),
        Threshold(threshold=3, callback=log_value),
    ])
    
    rclpy.init()
    node = Node("pipeline_node")

    msg_type = std_msg.Float32
    inp_topic_name = "/data_in"
    out_topic_name = "/data_out"

    flow = Dataflow()
    flow.input("inp", RosTopicInput(node, msg_type, inp_topic_name))
    flow.map(message_to_value)
    flow.filter_map(threshold_handler.select_threshold)
    flow.map(execute_threshold)
    flow.map(threshold_result_to_message)
    flow.output("out", RosTopicOutput(node, msg_type, out_topic_name))

    flow_args = {
        "flow": flow,
        "recovery_config": None,
    }

    # cli_main(**flow_args)
    
    thread1 = Thread(target=partial(cli_main, **flow_args))
    thread2 = Thread(target=partial(cli_main, **flow_args))
    thread1.start()
    thread2.start()
    
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()