import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Union

import matplotlib.animation as anim
import matplotlib.pyplot as plt
from cycler import cycler
from registrator import Subscriber


@dataclass
class PlotModel:
    node_name: str


class RecordedTrajectory:
    def __init__(self, name: str, x: list[float], y: list[float]):
        self.model = PlotModel(node_name=name)
        self._x = x
        self._y = y

    def get_trajectory_data(self):
        return self._x, self._y


def _collect_record_files(paths: Iterable[Union[str, Path]]) -> list[Path]:
    files: list[Path] = []
    for raw_path in paths:
        path = Path(raw_path)
        if path.is_dir():
            files.extend(sorted(path.glob("*.json")))
        elif path.exists():
            files.append(path)
    return files


def _load_recorded_trajectories(
    paths: Iterable[Union[str, Path]]
) -> list[RecordedTrajectory]:
    trajectories: list[RecordedTrajectory] = []
    for path in _collect_record_files(paths):
        with open(path, "r") as file:
            payload = json.load(file)

        record_name = payload.get("record_name") or path.stem
        data_points = payload.get("data", [])
        x = [point["position"]["x"] for point in data_points if "position" in point]
        y = [point["position"]["y"] for point in data_points if "position" in point]
        trajectories.append(RecordedTrajectory(name=record_name, x=x, y=y))
    return trajectories


def plot_2d_traj(
    *,
    subscribers: Optional[Sequence[Union[RecordedTrajectory, Subscriber]]] = None,
    interval: int = 100,
    recorded_paths: Optional[Union[str, Path, list[Union[str, Path]]]] = None,
):
    if recorded_paths:
        paths = recorded_paths if isinstance(recorded_paths, list) else [recorded_paths]
        subscribers = _load_recorded_trajectories(paths)

    fig, ax = plt.subplots()
    fig.patch.set_facecolor("white")

    # Set up color cycle for multiple trajectories
    colors = ["blue", "red", "orange", "purple", "green", "brown"]
    ax.set_prop_cycle(cycler("color", colors))

    def update_plot(_):
        ax.clear()
        ax.set_prop_cycle(cycler("color", colors))
        ax.set_aspect("equal", "box")
        ax.grid(True, linestyle="-.", alpha=0.3)
        _mx = []
        _my = []

        for subscriber in subscribers:
            x, y = subscriber.get_trajectory_data()
            if len(x) > 0 and len(y) > 0:
                ax.plot(x, y, label=subscriber.model.node_name, alpha=0.8)
                _mx.append(max(max(x), max(y)))
                _my.append(min(min(x), min(y)))

        max_val = max(_mx) if _mx else 0
        min_val = min(_my) if _my else 0

        ax.set_xlabel("x [m]", fontsize=12)
        ax.set_ylabel("y [m]", fontsize=12)
        ax.set_title("2D Trajectory Comparison", fontsize=14, fontweight="bold")
        ax.legend(loc="best", fontsize=11)
        axes = plt.gca()
        axes.set_xlim(min_val - 0.2, max_val + 0.2)
        axes.set_ylim(min_val - 0.2, max_val + 0.2)

        return (ax,)

    ani = anim.FuncAnimation(fig, update_plot, interval=interval, blit=False)
    plt.show()
