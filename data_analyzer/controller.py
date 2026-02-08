import argparse
import threading
from typing import List

import rclpy
from plotter import plot_2d_traj
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from registrator import create_pose_subscriber


class Controller:
    executor: MultiThreadedExecutor = None
    thread: threading.Thread = None
    _nodes: List[Node]

    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self._nodes = []

    def register(self, topic: str, name: str = "", timeout: float = 1.0):
        if not name:
            name = topic.replace("/", "_")

        _node = create_pose_subscriber(topic=topic, node_name=name, timeout=timeout)
        self._nodes.append(_node)
        self.executor.add_node(_node)
        print(f"Subcriber {_node} of type {_node.model.msg_type} is registered!")

    @property
    def nodes(self):
        return self._nodes

    def run(self):
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.thread.join()


def parse_args():
    parser = argparse.ArgumentParser("DataAnalyzer")
    parser.add_argument(
        "topics",
        nargs="+",
        help=(
            "Topics to listen provided in a format\n"
            "TOPIC:[NAME] TOPIC2:[NAME2]\n"
            "Example:\n"
            " amcl_pose:AMCL world odometry/filtered:EKF"
        ),
    )
    parser.add_argument(
        "--timeout",
        default=1.0,
        type=float,
        required=False,
        help="Timeout in seconds to register a topic",
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
    topics = get_topics(args)
    rclpy.init()
    controller = Controller()

    timeout = args.timeout if args.timeout else 1.0

    for topic, name in topics.items():
        controller.register(topic=topic, name=name, timeout=timeout)

    controller.run()
    plot_2d_traj(controller.nodes)

    rclpy.shutdown()
    controller.stop()


if __name__ == "__main__":
    main()
