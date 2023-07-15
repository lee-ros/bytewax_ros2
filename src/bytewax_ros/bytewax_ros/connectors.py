from typing import Any

import rclpy

from bytewax.inputs import StatelessSource, DynamicInput
from bytewax.outputs import StatelessSink, DynamicOutput
from rclpy.node import Node
from rclpy.utilities import try_shutdown as rclpy_try_shutdown


__all__ = [
    "RosTopicInput",
    "RosTopicOutput",
]


class _RosTopicSource(StatelessSource):
    def __init__(self, node: Node, msg_type: Any, topic_name: str):
        super().__init__()
        self._node = node
        self._msg = None
        
        node.create_subscription(msg_type, topic_name, self._fetch_message_from_topic, 10)
        
    def _fetch_message_from_topic(self, new_msg: Any):
        self._msg = new_msg
        
    def next(self) -> Any | None:
        self._run_node()
        
        temp_msg = self._msg
        self._msg = None
        
        return temp_msg
    
    def _run_node(self):
        rclpy.spin_once(self._node)
        
    def close(self):
        rclpy_try_shutdown()
    

class RosTopicInput(DynamicInput):
    def __init__(self, node: Node, msg_type: Any, topic_name: str):
        super().__init__()
        self._node = node
        self._msg_type = msg_type
        self._topic = topic_name
        
    def build(self, worker_index, worker_count):
        return _RosTopicSource(self._node, self._msg_type, self._topic)
    
    
class _RosTopicSink(StatelessSink):
    def __init__(self, node: Node, msg_type: Any, topic_name: str):
        super().__init__()
        self._publisher = node.create_publisher(msg_type, topic_name, 10)
        
    def write(self, item: Any):
        self._publisher.publish(item)
        
    def close(self):
        rclpy_try_shutdown()


class RosTopicOutput(DynamicOutput):
    def __init__(self, node: Node, msg_type: Any, topic: str):
        super().__init__()
        self._node = node
        self._msg_type = msg_type
        self._topic = topic
        
    def build(self, worker_index, worker_count):
        return _RosTopicSink(self._node, self._msg_type, self._topic)