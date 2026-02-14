from dataclasses import dataclass
from typing import Union


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
class AngularVelocityData:
    x: float
    y: float
    z: float


@dataclass
class LinearAccelerationData:
    x: float
    y: float
    z: float


@dataclass
class Data:
    timestamp: float
    position: PositionData
    orientation: OrientationData


@dataclass
class ImuData(Data):
    angular_velocity: AngularVelocityData
    linear_acceleration: LinearAccelerationData


@dataclass
class DataModel:
    record_name: str
    data: list[Union[Data, ImuData]]
