import argparse
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import List, Optional, Union

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

from .registrator import create_pose_subscriber, create_imu_subscriber, Subscriber
from .recorder import Recorder, create_recorder
from .plotter import plot_2d_traj, plot_imu_data
from .watchdog import WatchdogTimer


class Controller:
    executor: MultiThreadedExecutor = None
    thread: threading.Thread = None
    _nodes: List[Subscriber]
    _recorders: List[Recorder]
    _logger: "RcutilsLogger"
    _wd: WatchdogTimer
    _kick_pid_list: Optional[List[int]]

    def __init__(self, wd_timeout: float = 5.0, watchdog_ignore: List[str] = None):
        self.executor = MultiThreadedExecutor()
        self._nodes = []
        self._recorders = []
        self._kick_pid_list = []
        self._logger = get_logger(__name__)
        self._wd = WatchdogTimer(
            timeout=wd_timeout,
            timeout_handler=self.save_all,
            send_signal=False,
        )
        self._watchdog_ignore = self._normalize_watchdog_ignore(watchdog_ignore)

    def _normalize_watchdog_ignore(
        self, watchdog_ignore: Optional[List[str]]
    ) -> set[str]:
        if not watchdog_ignore:
            return set()
        normalized: set[str] = set()
        for item in watchdog_ignore:
            if not item:
                continue
            for part in item.split(","):
                part = part.strip()
                if not part:
                    continue
                normalized.add(part)
                normalized.add(part.lstrip("/"))
                normalized.add(f"/{part.lstrip('/')}")
        return normalized

    def _skip_watchdog(self, topic: str) -> bool:
        return topic in self._watchdog_ignore

    def register(self, topic: str, name: str = "", timeout: float = 1.0):
        if not name:
            name = topic.replace("/", "_")

        _node = create_pose_subscriber(topic=topic, node_name=name, timeout=timeout)
        if not self._skip_watchdog(topic):
            _node.set_watchdog(self._wd)
        self._nodes.append(_node)
        self.executor.add_node(_node)
        self._logger.info(
            f"Subscriber {_node} of type {_node.model.msg_type} is registered!"
        )
        return _node

    def register_imu(self, topic: str, name: str = "", timeout: float = 1.0):
        if not name:
            name = topic.replace("/", "_")

        _node = create_imu_subscriber(topic=topic, node_name=name)
        if not self._skip_watchdog(topic):
            _node.set_watchdog(self._wd)
        self._nodes.append(_node)
        self.executor.add_node(_node)
        self._logger.info(
            f"Subscriber {_node} for topic {topic} of type {_node.model.msg_type} is registered!"
        )
        return _node

    @property
    def nodes(self):
        return self._nodes

    def run(self):
        self._wd.start()
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self._wd.stop()
        self.thread.join()

    def set_watchdog_timeout_handler(self, timeout_handler) -> None:
        self._wd.set_timeout_handler(timeout_handler)

    def add_pid_to_kick(self, pid: Union[int, List[int]]) -> None:
        if isinstance(pid, int):
            self._kick_pid_list.append(pid)
        else:
            self._kick_pid_list.extend(pid)

    def _terminate_pids(self) -> None:
        if self._kick_pid_list:
            for pid in self._kick_pid_list:
                try:
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(2)
                    try:
                        os.kill(pid, 0)
                        self._logger.warning(f"Process {pid} still alive, killing...")
                        os.kill(pid, signal.SIGKILL)
                    except OSError:
                        self._logger.debug(f"Process {pid} is successfully terminated.")
                except (ValueError, ProcessLookupError, Exception) as e:
                    self._logger.debug(f"Could not terminate process {pid}: {e}")

    def add_recorder(self, node: Subscriber, target_path: str):
        target_file = self._resolve_target_file(node=node, target_path=target_path)
        recorder = create_recorder(node=node, target_file=target_file)
        node.add_data_callback(recorder.update_data)
        self._recorders.append(recorder)

    def save_all(self):
        for recorder in self._recorders:
            recorder.save_data()

    def _resolve_target_file(self, node: Subscriber, target_path: str) -> Path:
        path = Path(target_path)
        is_json_file = path.suffix.lower() == ".json" and not path.is_dir()
        multiple_nodes = len(self._nodes) > 1

        if is_json_file and not multiple_nodes:
            return path

        if is_json_file and multiple_nodes:
            return path.with_name(f"{path.stem}_{node.get_name()}{path.suffix}")

        return path / f"{node.get_name()}.json"


def parse_args():
    parser = argparse.ArgumentParser("DataAnalyzer")
    parser.add_argument(
        "--log_level",
        default="INFO",
        type=str,
        required=False,
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    record_parser = subparsers.add_parser(
        "record", help="Subscribe to topics and optionally plot or record"
    )
    record_parser.add_argument(
        "topics",
        nargs="+",
        help=(
            "Topics to listen provided in a format\n"
            "TOPIC:[NAME] TOPIC2:[NAME2]\n"
            "Example:\n"
            " amcl_pose:AMCL world odometry/filtered:EKF"
        ),
    )
    record_parser.add_argument(
        "--timeout",
        default=1.0,
        type=float,
        required=False,
        help="Timeout in seconds to register a topic",
    )
    record_parser.add_argument(
        "--plot",
        action="store_true",
        default=False,
        required=False,
        help="Plot the trajectories after processing the data",
    )
    record_parser.add_argument(
        "--record_to",
        default=None,
        type=str,
        required=False,
        help="Path to save the recorded data in JSON format",
    )
    record_parser.add_argument(
        "--imu",
        type=str,
        required=False,
        help="Additional topic for IMU data in the format TOPIC:[NAME]",
    )
    record_parser.add_argument(
        "--watchdog_ignore",
        nargs="*",
        default=[],
        help=("Topics that should not ping the watchdog (space- or comma-separated)."),
    )
    record_parser.add_argument(
        "--watchdog_timeout",
        default=5.0,
        type=float,
        required=False,
        help="Timeout in seconds for the watchdog to trigger if no pings are received",
    )
    record_parser.add_argument(
        "--follow_pids",
        default=None,
        type=int,
        required=False,
        help="PID of the path_publisher process to terminate on watchdog timeout",
    )

    plot_parser = subparsers.add_parser(
        "plot", help="Plot trajectories from recorded data"
    )
    plot_parser.add_argument(
        "paths",
        nargs="+",
        help="Path(s) to recorded data files or directories",
    )

    return parser.parse_args()


def get_topics(arg):
    if isinstance(arg, str):
        arg = [arg]
    topics = {}
    for topic in arg:
        try:
            _t, _n = topic.split(":")
        except ValueError:
            _t = topic
            _n = topic
        topics[_t] = _n
    return topics


def main():
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    if args.command == "plot":
        plot_2d_traj(recorded_paths=args.paths)
        plot_imu_data(recorded_paths=args.paths)
        return

    topics = get_topics(args.topics)
    rclpy.init()
    controller = Controller(
        watchdog_ignore=args.watchdog_ignore, wd_timeout=args.watchdog_timeout
    )
    stop_event = threading.Event()
    shutdown_lock = threading.Lock()
    shutdown_done = False

    def save_once():
        nonlocal shutdown_done
        with shutdown_lock:
            if shutdown_done:
                return
            shutdown_done = True
        if args.record_to:
            controller.save_all()

    def on_rclpy_shutdown():
        stop_event.set()
        save_once()

    def handle_signal(_signum, _frame):
        stop_event.set()
        save_once()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if args.plot:
            try:
                import matplotlib.pyplot as plt

                plt.close("all")
            except Exception:
                pass

    rclpy.get_default_context().on_shutdown(on_rclpy_shutdown)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    def on_watchdog_timeout():
        controller._terminate_pids()
        save_once()
        stop_event.set()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    controller.set_watchdog_timeout_handler(on_watchdog_timeout)

    timeout = args.timeout

    for topic, name in topics.items():
        controller.register(topic=topic, name=name, timeout=timeout)

    if args.imu:
        imu_topic, imu_name = get_topics(args.imu).popitem()
        controller.register_imu(topic=imu_topic, name=imu_name, timeout=timeout)

    if args.record_to:
        for node in controller.nodes:
            controller.add_recorder(node=node, target_path=args.record_to)

    if args.follow_pids:
        controller.add_pid_to_kick(args.follow_pids)

    controller.run()
    plot_thread = None
    if args.plot:
        plot_thread = threading.Thread(
            target=plot_2d_traj,
            kwargs={"subscribers": controller.nodes},
            daemon=True,
        )
        plot_thread.start()

    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass
    finally:
        save_once()
        if args.plot:
            try:
                import matplotlib.pyplot as plt

                plt.close("all")
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        controller.stop()


if __name__ == "__main__":
    main()
