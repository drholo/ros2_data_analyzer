import argparse
import threading
import time
from pathlib import Path
from typing import List

import rclpy
from plotter import plot_2d_traj
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from registrator import create_pose_subscriber
from recorder import DataModel, Recorder, RecorderModel


class Controller:
    executor: MultiThreadedExecutor = None
    thread: threading.Thread = None
    _nodes: List[Node]
    _recorders: List[Recorder]

    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self._nodes = []
        self._recorders = []

    def register(self, topic: str, name: str = "", timeout: float = 1.0):
        if not name:
            name = topic.replace("/", "_")

        _node = create_pose_subscriber(topic=topic, node_name=name, timeout=timeout)
        self._nodes.append(_node)
        self.executor.add_node(_node)
        print(f"Subcriber {_node} of type {_node.model.msg_type} is registered!")
        return _node

    @property
    def nodes(self):
        return self._nodes

    def run(self):
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.thread.join()

    def add_recorder(self, node: Node, target_path: str):
        target_file = self._resolve_target_file(node=node, target_path=target_path)
        data_model = DataModel(record_name=node.get_name(), data=[])
        recorder_model = RecorderModel(
            recorder_name=f"recorder_{node.get_name()}",
            target_file=target_file,
            subscriber=node,
            data=data_model,
        )
        recorder = Recorder(recorder_model)
        node.add_data_callback(recorder.update_data)
        self._recorders.append(recorder)

    def save_all(self):
        for recorder in self._recorders:
            recorder.save_data()

    def _resolve_target_file(self, node: Node, target_path: str) -> Path:
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

    plot_parser = subparsers.add_parser(
        "plot", help="Plot trajectories from recorded data"
    )
    plot_parser.add_argument(
        "paths",
        nargs="+",
        help="Path(s) to recorded data files or directories",
    )

    return parser.parse_args()


def get_topics(args):
    topics = {}
    for topic in args.topics:
        try:
            _t, _n = topic.split(":")
        except ValueError:
            _t = topic
            _n = topic
        topics[_t] = _n
    return topics


def main():
    args = parse_args()
    if args.command == "plot":
        plot_2d_traj([], recorded_paths=args.paths)
        return

    topics = get_topics(args)
    rclpy.init()
    controller = Controller()

    timeout = args.timeout if args.timeout else 1.0

    for topic, name in topics.items():
        controller.register(topic=topic, name=name, timeout=timeout)

    if args.record_to:
        for node in controller.nodes:
            controller.add_recorder(node=node, target_path=args.record_to)

    controller.run()
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if args.record_to:
            controller.save_all()
        rclpy.shutdown()
        controller.stop()

    if args.plot:
        if args.record_to:
            plot_2d_traj(controller.nodes, recorded_paths=args.record_to)
        else:
            plot_2d_traj(controller.nodes)


if __name__ == "__main__":
    main()
