from dataclasses import dataclass
from threading import Lock
from typing import Callable, Union, override

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@dataclass
class NodeModel:
    node_name: str


@dataclass
class SubscriberModel(NodeModel):
    topic: str
    msg_type: Type[Union[PoseWithCovarianceStamped, Odometry]]
    negate_xy: bool = False


@dataclass
class TransformSubscriberModel(NodeModel):
    target_frame: str
    source_frame: str


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

    def run_callback(self, msg):
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


class TransformSubscriber(Node):
    def __init__(
        self,
        target_frame: str,
        source_frame: str,
        node_name: str = "transform_subscriber",
    ):
        self.model = TransformSubscriberModel(
            node_name=node_name,
            target_frame=target_frame,
            source_frame=source_frame,
        )
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.cb_timer)

    def cb_timer(self):
        from_frame = self.model.source_frame
        to_frame = self.model.target_frame
        current_time = Time()

        try:
            tf_data = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                current_time,
            )
            translation = tf_data.transform.translation
            rotation = tf_data.transform.rotation
            self.get_logger().info(
                f"Translation: x={translation.x:.2f}, y={translation.y:.2f}, z={translation.z:.2f} | "
                f"Rotation: x={rotation.x:.2f}, y={rotation.y:.2f}, z={rotation.z:.2f}, w={rotation.w:.2f}"
            )
            self.cb_data_process(tf_data)
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {from_frame} to {to_frame}: {ex}"
            )
            return

    def cb_data_process(self, tf_data: TransformStamped):
        pass


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
