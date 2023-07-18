from enum import Enum, auto


class BandSeverity(Enum):
    OK = auto()
    WARN = auto()
    ERROR = auto()
    CRITICAL = auto()
