from typing import Any

import rclpy

from bytewax.inputs import StatelessSource, DynamicInput
from bytewax.outputs import StatelessSink, DynamicOutput
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok


__all__ = [
    "RosTopicInput",
    "RosTopicOutput",
]


def _create_node_with_context(name: str):
        if not rclpy_ok():
            rclpy.init()
        return Node(name)


class _RosTopicSource(StatelessSource):
    def __init__(self, msg_type: Any, topic: str):
        super().__init__()
        self._node = _create_node_with_context("pipeline_subscriber")
        self._node.create_subscription(msg_type, topic, self._fetch_message_from_topic, 10)
        
        self._msg = None
    
        
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
        rclpy.shutdown()
    

class RosTopicInput(DynamicInput):
    def __init__(self, msg_type: Any, topic: str):
        super().__init__()
        self._msg_type = msg_type
        self._topic = topic
        
    def build(self, worker_index, worker_count):
        return _RosTopicSource(self._msg_type, self._topic)
    
    
class _RosTopicSink(StatelessSink):
    def __init__(self, msg_type: Any, topic_name: str):
        super().__init__()
        node = _create_node_with_context("pipeline_publisher")
        self._publisher = node.create_publisher(msg_type, topic_name, 10)
        
    def write(self, item: Any):
        self._publisher.publish(item)
        
    def close(self):
        rclpy.shutdown()


class RosTopicOutput(DynamicOutput):
    def __init__(self, msg_type: Any, topic: str):
        super().__init__()
        self._msg_type = msg_type
        self._topic = topic
        
    def build(self, worker_index, worker_count):
        return _RosTopicSink(self._msg_type, self._topic)