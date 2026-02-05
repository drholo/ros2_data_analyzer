import argparse
import threading
from typing import List

import rclpy
from plotter import plot_2d_traj
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from registrator import OdometrySubscriber, PoseSubscriber


class Controller:
    executor: MultiThreadedExecutor
    thread: threading.Thread
    _nodes: List[Node]

    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self._nodes = []

    def register(
        self, topic: str, name: str = "", pose: bool = False, negate: bool = False
    ):
        if not name:
            name = topic.replace("/", "_")
        if pose:
            _node = PoseSubscriber(topic=topic, node_name=name, negate_xy=negate)
        else:
            _node = OdometrySubscriber(topic=topic, node_name=name, negate_xy=negate)
        self._nodes.append(_node)
        self.executor.add_node(_node)

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

    for topic, name in topics.items():
        if name == "world":
            controller.register(topic, "simulator", negate=True)
        else:
            controller.register(topic, name)

    controller.run()
    plot_2d_traj(controller.nodes)

    rclpy.shutdown()
    controller.stop()


if __name__ == "__main__":
    main()
