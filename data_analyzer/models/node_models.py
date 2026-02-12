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
