from time import sleep

import rclpy

from bytewax.dataflow import Dataflow
from std_msgs import msg as std_msg
from rclpy.node import Node
from rclpy.utilities import try_shutdown as rclpy_shutdown

from bytewax_ros.connectors import RosTopicInput, RosTopicOutput
from bytewax_ros.utils import run_flow_as_thread, value_from_obj


def main():
    rclpy.init()
    node = Node("flow_node")  # type: ignore
    
    inp_msg_type = std_msg.Float32
    inp_topic_name = "/data_in"
    
    out_msg_type = std_msg.String
    out_topic_name = "/data_out"
    
    flow = Dataflow()
    flow.input("inp", RosTopicInput(node, inp_msg_type, inp_topic_name))
    flow.map(lambda msg: value_from_obj(msg, "obj.data"))
    flow.map(lambda data: std_msg.String(data=str(data)))
    flow.output("out", RosTopicOutput(node, out_msg_type, out_topic_name))

    run_flow_as_thread("flow", flow)

    while True:
        try:
            rclpy.spin_once(node)
            sleep(0)
        except KeyboardInterrupt:
            break

    rclpy_shutdown()


if __name__ == "__main__":
    main()
