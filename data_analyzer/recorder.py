from dataclasses import dataclass
from pathlib import Path
import json
import signal
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from registrator import Subscriber

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


class Recorder:
    def __init__(self, model: RecorderModel):
        self.model = model
        self.subscriber = self.model.subscriber
        self._previous_sigint_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._handle_sigint)
        if self.model.data.data is None:
            self.data = []
        else:            
            self.data = self.model.data.data
    
    def update_data(self, data: Data):
        self.data.append(data)

    def save_data(self):
        data_model = DataModel(record_name=self.model.data.record_name, data=self.data)
        with open(self.model.target_file, "w") as f:
            json.dump(data_model.__dict__, f, default=lambda o: o.__dict__, indent=4)

    def _handle_sigint(self, signum, frame):
        self.save_data()
        if callable(self._previous_sigint_handler) and self._previous_sigint_handler is not self._handle_sigint:
            self._previous_sigint_handler(signum, frame)
            return
        raise KeyboardInterrupt
