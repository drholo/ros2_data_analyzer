import json

try:
    from .models import Data, DataModel, RecorderModel
except ImportError:
    from models import Data, DataModel, RecorderModel
from rclpy.logging import get_logger


class Recorder:
    def __init__(self, model: RecorderModel):
        self.model = model
        self.subscriber = self.model.subscriber
        self._logger = get_logger(__name__)
        if self.model.data.data is None:
            self.data = []
        else:
            self.data = self.model.data.data
        self._logger.info(f"{self.model.recorder_name} is initialized")

    def update_data(self, data: Data):
        self.data.append(data)

    def save_data(self):
        data_model = DataModel(record_name=self.model.data.record_name, data=self.data)
        self._logger.info(f"Saving data to {self.model.target_file}...")
        with open(self.model.target_file, "w") as _file:
            json.dump(
                data_model.__dict__, _file, default=lambda obj: obj.__dict__, indent=4
            )


def create_recorder(node, target_file, data=None) -> Recorder:
    if data is None:
        data = []
    data_model = DataModel(record_name=node.get_name(), data=data)
    recorder_model = RecorderModel(
        recorder_name=f"recorder_{node.get_name()}",
        target_file=target_file,
        subscriber=node,
        data=data_model,
    )
    return Recorder(model=recorder_model)
