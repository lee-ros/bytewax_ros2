from bytewax.connectors.stdio import StdOutput
from bytewax.dataflow import Dataflow
from std_msgs import msg as std_msg

from .connectors import RosTopicInput, RosTopicOutput


msg_type = std_msg.String
inp_topic_name = "/data_in"
out_topic_name = "/data_out"

flow = Dataflow()
flow.input("inp", RosTopicInput(msg_type, inp_topic_name))
flow.map(lambda msg: msg.data)
flow.output("out", StdOutput())
flow.map(lambda val: std_msg.String(data=val + "!"))
flow.output("out", RosTopicOutput(msg_type, out_topic_name))
