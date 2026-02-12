from dataclasses import dataclass


@dataclass
class PositionData:
    x: float
    y: float
    z: float


@dataclass
class OrientationData:
    x: float
    y: float
    z: float
    w: float


@dataclass
class OrientationEulerData:
    roll: float
    pitch: float
    yaw: float


@dataclass
class Data:
    timestamp: float
    position: PositionData
    orientation: OrientationData


@dataclass
class DataModel:
    record_name: str
    data: list[Data]
