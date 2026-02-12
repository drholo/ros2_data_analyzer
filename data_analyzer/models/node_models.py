from dataclasses import dataclass
from typing import TYPE_CHECKING, Type, Union
from pathlib import Path

from .data_models import DataModel

if TYPE_CHECKING:
    from ..registrator import Subscriber


@dataclass
class RecorderModel:
    recorder_name: str
    target_file: Union[str, Path]
    subscriber: "Subscriber"
    data: DataModel

    def __post_init__(self):
        if not self.recorder_name:
            self.recorder_name = f"recorder_{self.subscriber.model.node_name}"
        if not self.target_file:
            self.target_file = Path(f"data/{self.recorder_name}.json")
        if isinstance(self.target_file, str):
            self.target_file = Path(self.target_file)
        if not self.target_file.parent.exists():
            self.target_file.parent.mkdir(parents=True, exist_ok=True)


@dataclass
class PlotModel:
    node_name: str
    data: DataModel


@dataclass
class ImuPlotModel(PlotModel):
    timestamps: list[float]
    orientation: list[tuple]
    angular_velocity: list[tuple]
    linear_acceleration: list[tuple]

    def __post_init__(self):
        if not self.orientation:
            self.orientation = []
        if not self.timestamps:
            self.timestamps = []
        if not self.angular_velocity:
            self.angular_velocity = []
        if not self.linear_acceleration:
            self.linear_acceleration = []


def get_acceleration(msg):
    return (
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
    )


def get_angular_velocity(msg):
    return msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z


def get_orientation(msg):
    return msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w


@dataclass
class TrajectoryPlotModel(PlotModel):
    x: list[float]
    y: list[float]


@dataclass
class NodeModel:
    node_name: str


@dataclass
class SubscriberModel(NodeModel):
    topic: str
    msg_type: Type


@dataclass
class TransformSubscriberModel(NodeModel):
    target_frame: str
    source_frame: str
    timer: float = 0.1
