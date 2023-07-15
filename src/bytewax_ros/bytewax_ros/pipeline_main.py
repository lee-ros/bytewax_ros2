import rclpy

from bytewax.connectors.stdio import StdOutput
from bytewax.dataflow import Dataflow
from bytewax.run import cli_main
from std_msgs import msg as std_msg
from rclpy.node import Node

from connectors import RosTopicInput, RosTopicOutput


def main():
    rclpy.init()
    node = Node("pipeline_node")

    msg_type = std_msg.String
    inp_topic_name = "/data_in"
    out_topic_name = "/data_out"

    flow = Dataflow()
    flow.input("inp", RosTopicInput(node, msg_type, inp_topic_name))
    flow.map(lambda msg: msg.data)
    flow.output("out", StdOutput())
    flow.map(lambda val: std_msg.String(data=val + "!"))
    flow.output("out", RosTopicOutput(node, msg_type, out_topic_name))

    flow_args = {
        "flow": flow,
        "recovery_config": None,
    }

    cli_main(**flow_args)

    rclpy.shutdown()


if __name__ == "__main__":
    main()