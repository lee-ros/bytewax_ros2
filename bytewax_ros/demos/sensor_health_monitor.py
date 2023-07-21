import math
from time import sleep
import uuid
import rclpy

from bytewax.dataflow import Dataflow
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Float32
from rclpy.node import Node
from rclpy.utilities import try_shutdown as rclpy_shutdown

from bytewax_ros.bands import Band, BandDetector, BandSeverity
from bytewax_ros.connectors import RosTopicInput, RosTopicOutput
from bytewax_ros.execution import run_flow_as_thread
from bytewax_ros.unique import UniqueFilter


def band_to_diagnostic(band: Band) -> DiagnosticStatus:
    return DiagnosticStatus(
        level=DiagnosticStatus.ERROR,
        name="Sensor1 Health",
        message=f"Sensor1 health is at: {band.severity}",
        hardware_id="sensor_#1",
        values=[],
    )


def create_flow(node: Node) -> Dataflow:
    flow = Dataflow()

    inp_topic = "/sensor1/data"
    inp_type = Float32
    flow.input("inp", RosTopicInput(node, inp_type, inp_topic))
    flow.map(lambda ros_msg: ros_msg.data)

    bands = [
        Band(4, math.inf, BandSeverity.CRITICAL),
        Band(2, 4, BandSeverity.ERROR),
        Band(-2, 2, BandSeverity.OK),
        Band(-math.inf, -2, BandSeverity.WARN),
    ]
    detector = BandDetector(bands, verify_no_gaps=True)
    flow.filter_map(detector.detect_band)

    unique_filter = UniqueFilter()
    flow.filter(unique_filter)
    flow.map(band_to_diagnostic)

    out_topic = "/sensors_health"
    out_type = DiagnosticStatus
    flow.output("out", RosTopicOutput(node, out_type, out_topic))

    return flow


def main():
    rclpy.init()

    node = Node("sensor_health")
    flow = create_flow(node)
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
