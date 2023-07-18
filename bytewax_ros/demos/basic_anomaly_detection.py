from bytewax.connectors.stdio import StdOutput
from bytewax.dataflow import Dataflow
from bytewax.testing import  TestingInput

from bytewax_ros.bands import BandDetector, BandSeverity, DefaultBand, SampleTriggeredBand
from bytewax_ros.utilities import run_flow


critical_above_band = SampleTriggeredBand(5, 10, BandSeverity.CRITICAL, 1)
warn_above_band = SampleTriggeredBand(2, 4, BandSeverity.WARN, 3)
default_band = DefaultBand(BandSeverity.OK)

anomaly_detector = BandDetector(
    [critical_above_band, warn_above_band],
    default_band=default_band
)

inp = [0] * 5 + [3] * 5 + [8] * 10

flow = Dataflow()
flow.input("inp", TestingInput(inp))
flow.map(anomaly_detector.detect_band)
flow.output("out", StdOutput())

run_flow(flow)
