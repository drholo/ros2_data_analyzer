from dataclasses import dataclass
from typing import Union, Callable, override
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from threading import Lock


@dataclass
class SubscriberModel:
    topic: str
    msg_type: Union[PoseWithCovarianceStamped, Odometry]
    node_name: str = "subscriber_node"
    negate_xy: bool = False  # Negate both x and y if True


@dataclass
class PoseSubscriberModel(SubscriberModel):
    msg_type: Callable = PoseWithCovarianceStamped


@dataclass
class OdometrySubscriberModel(SubscriberModel):
    msg_type: Callable = Odometry


class Subscriber(Node):
    def __init__(self, model: SubscriberModel):
        self.model = model
        self.x = []
        self.y = []
        super().__init__(model.node_name)
        self.subscription = self.create_subscription(
            model.msg_type, model.topic, self.run_callback, 10
        )
        self._lock = Lock()

    def get_pose(self, msg):
        if isinstance(msg, PoseWithCovarianceStamped) or isinstance(msg, Odometry):
            return msg.pose.pose
        else:
            self.get_logger().error(
                "Message is not of type PoseWithCovarianceStamped or Odometry"
            )
            return None

    def run_callback(self, msg: Union[PoseWithCovarianceStamped, Odometry]):
        raise NotImplementedError

    def get_trajectory_data(self):
        with self._lock:
            return self.x, self.y

    def update_data(self, pose):
        with self._lock:
            x, y, z = get_coordinates(pose)
            # if simulator data
            if self.model.negate_xy:
                x = -x
                y = -y
            self.get_logger().debug(f"Coordinates: {x} {y} {z}")
            self.x.append(x)
            self.y.append(y)


class PoseSubscriber(Subscriber):
    def __init__(self, topic: str, node_name: str = "", negate_xy: bool = False):
        super().__init__(
            PoseSubscriberModel(topic=topic, node_name=node_name, negate_xy=negate_xy)
        )

    @override
    def run_callback(self, msg: PoseWithCovarianceStamped):
        pose = self.get_pose(msg)
        if pose:
            self.get_logger().debug(
                f"Received {self.model.msg_type.__name__} message from topic {self.model.topic}"
            )
            self.get_logger().debug(
                f"Position: {get_position(pose)}, Orientation: {get_orientation(pose)}"
            )
            self.update_data(pose)


class OdometrySubscriber(Subscriber):
    def __init__(self, topic: str, node_name: str = "", negate_xy: bool = False):
        super().__init__(
            OdometrySubscriberModel(
                topic=topic, node_name=node_name, negate_xy=negate_xy
            )
        )

    def get_twist(self, msg):
        return msg.twist.twist

    @override
    def run_callback(self, msg: Odometry):
        self.get_logger().debug(
            f"Received {self.model.msg_type.__name__} message from topic {self.model.topic}"
        )
        twist = self.get_twist(msg)
        pose = self.get_pose(msg)
        if pose:
            self.get_logger().debug(
                f"Position: {get_position(pose)}, Orientation: {get_orientation(pose)}"
            )
            self.update_data(pose)
        if twist:
            self.get_logger().debug(f"Linear: {twist.linear}, Angular: {twist.angular}")


def get_position(pose):
    if pose:
        return pose.position
    return None


def get_orientation(pose):
    if pose:
        return pose.orientation
    return None


def get_coordinates(pose):
    return pose.position.x, pose.position.y, pose.position.z


def get_quaternions(orientation):
    return orientation.x, orientation.y, orientation.z, orientation.w
