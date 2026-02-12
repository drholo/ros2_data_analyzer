import json
import math
from pathlib import Path
from typing import Iterable, Optional, Sequence, Union
from xml.parsers.expat import model

import matplotlib.animation as anim
import matplotlib.pyplot as plt
from cycler import cycler

from .registrator import Subscriber
from .models import ImuPlotModel, TrajectoryPlotModel


class RecorderIMU:
    def __init__(
        self,
        name: str,
        timestamps: list[float],
        orientations: list[tuple],
        angular_velocity: list[tuple],
        linear_acceleration: list[tuple],
    ):
        self.model = ImuPlotModel(
            node_name=name,
            timestamps=timestamps,
            orientation=orientations,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
        )
        self._timestamps = timestamps
        self._orientations = orientations

    def get_trajectory_data(self):
        return self._timestamps, self._orientations


class RecordedTrajectory:
    def __init__(self, name: str, x: list[float], y: list[float]):
        self.model = TrajectoryPlotModel(node_name=name, x=x, y=y)
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
        if "imu" in path.stem.lower():
            # dirty fix for skipping IMU data files when loading trajectories
            continue
        with open(path, "r") as file:
            payload = json.load(file)

        record_name = payload.get("record_name") or path.stem
        data_points = payload.get("data", [])
        x = [point["position"]["x"] for point in data_points if "position" in point]
        y = [point["position"]["y"] for point in data_points if "position" in point]
        trajectories.append(RecordedTrajectory(name=record_name, x=x, y=y))
    return trajectories


def _load_recorded_imu_data(paths: Iterable[Union[str, Path]]) -> list[RecorderIMU]:
    imu_data_list: list[RecorderIMU] = []
    for path in _collect_record_files(paths):
        with open(path, "r") as file:
            payload = json.load(file)
        record_name = payload.get("record_name") or path.stem
        data_points = payload.get("data", [])
        timestamps = [
            point["timestamp"] for point in data_points if "timestamp" in point
        ]
        orientations = [
            _quaternion_to_euler(
                point["orientation"]["x"],
                point["orientation"]["y"],
                point["orientation"]["z"],
                point["orientation"]["w"],
            )
            for point in data_points
            if "orientation" in point
        ]
        angular_velocity = [
            (
                point["angular_velocity"]["x"],
                point["angular_velocity"]["y"],
                point["angular_velocity"]["z"],
            )
            for point in data_points
            if "angular_velocity" in point
        ]
        linear_acceleration = [
            (
                point["linear_acceleration"]["x"],
                point["linear_acceleration"]["y"],
                point["linear_acceleration"]["z"],
            )
            for point in data_points
            if "linear_acceleration" in point
        ]
        imu_data_list.append(
            RecorderIMU(
                name=record_name,
                timestamps=timestamps,
                orientations=orientations,
                angular_velocity=angular_velocity,
                linear_acceleration=linear_acceleration,
            )
        )
    return imu_data_list


def plot_2d_traj(
    *,
    subscribers: Optional[Sequence[Union[RecordedTrajectory, Subscriber]]] = None,
    interval: int = 100,
    recorded_paths: Optional[Union[str, Path, list[Union[str, Path]]]] = None,
):
    use_animation = False
    if recorded_paths:
        paths = recorded_paths if isinstance(recorded_paths, list) else [recorded_paths]
        subscribers = _load_recorded_trajectories(paths)
    else:
        use_animation = True

    fig, ax = plt.subplots()
    fig.patch.set_facecolor("white")

    # Set up color cycle for multiple trajectories
    colors = ["blue", "red", "orange", "purple", "green", "brown"]
    ax.set_prop_cycle(cycler("color", colors))

    def update_plot():
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

    if use_animation:
        ani = anim.FuncAnimation(
            fig, lambda _: update_plot(), interval=interval, blit=False
        )
    else:
        update_plot()
    plt.show()


def plot_imu_data(
    *,
    subscribers: Optional[Sequence[Union[RecorderIMU, Subscriber]]] = None,
    recorded_paths: Optional[Union[str, Path, list[Union[str, Path]]]] = None,
):
    if recorded_paths:
        paths = recorded_paths if isinstance(recorded_paths, list) else [recorded_paths]
        subscribers = _load_recorded_imu_data(paths)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    fig.patch.set_facecolor("white")

    for subscriber in subscribers:
        timestamps = subscriber.model.timestamps
        orientations = subscriber.model.orientation
        angular_velocity = subscriber.model.angular_velocity
        linear_acceleration = subscriber.model.linear_acceleration
        or_axes = axes[0]
        or_axes.plot(
            timestamps,
            [o[0] for o in orientations],
            label=f"{subscriber.model.node_name} - Roll",
        )
        or_axes.plot(
            timestamps,
            [o[1] for o in orientations],
            label=f"{subscriber.model.node_name} - Pitch",
        )
        or_axes.plot(
            timestamps,
            [o[2] for o in orientations],
            label=f"{subscriber.model.node_name} - Yaw",
        )
        av_axes = axes[1]
        av_axes.plot(
            timestamps,
            [av[0] for av in angular_velocity],
            label=f"{subscriber.model.node_name} - Angular Velocity X",
        )
        av_axes.plot(
            timestamps,
            [av[1] for av in angular_velocity],
            label=f"{subscriber.model.node_name} - Angular Velocity Y",
        )
        av_axes.plot(
            timestamps,
            [av[2] for av in angular_velocity],
            label=f"{subscriber.model.node_name} - Angular Velocity Z",
        )
        la_axes = axes[2]
        la_axes.plot(
            timestamps,
            [la[0] for la in linear_acceleration],
            label=f"{subscriber.model.node_name} - Linear Acceleration X",
        )
        la_axes.plot(
            timestamps,
            [la[1] for la in linear_acceleration],
            label=f"{subscriber.model.node_name} - Linear Acceleration Y",
        )
        la_axes.plot(
            timestamps,
            [la[2] for la in linear_acceleration],
            label=f"{subscriber.model.node_name} - Linear Acceleration Z",
        )

    axes[0].set_title("IMU Orientation Over Time", fontsize=14, fontweight="bold")
    axes[0].set_xlabel("Time [s]", fontsize=12)
    axes[0].set_ylabel("Orientation (radians)", fontsize=12)
    axes[0].legend(loc="best", fontsize=11)
    axes[0].grid(True, linestyle="-.", alpha=0.3)

    axes[1].set_title("IMU Angular Velocity Over Time", fontsize=14, fontweight="bold")
    axes[1].set_xlabel("Time [s]", fontsize=12)
    axes[1].set_ylabel("Angular Velocity (rad/s)", fontsize=12)
    axes[1].legend(loc="best", fontsize=11)
    axes[1].grid(True, linestyle="-.", alpha=0.3)

    axes[2].set_title(
        "IMU Linear Acceleration Over Time", fontsize=14, fontweight="bold"
    )
    axes[2].set_xlabel("Time [s]", fontsize=12)
    axes[2].set_ylabel("Linear Acceleration (m/s²)", fontsize=12)
    axes[2].legend(loc="best", fontsize=11)
    axes[2].grid(True, linestyle="-.", alpha=0.3)

    plt.tight_layout()
    plt.show()


def _quaternion_to_euler(q_w, q_x, q_y, q_z):

    # Roll (x-axis rotation)
    roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))

    # Pitch (y-axis rotation)
    pitch = math.asin(2 * (q_w * q_y - q_z * q_x))

    # Yaw (z-axis rotation)
    yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))

    return (roll, pitch, yaw)
