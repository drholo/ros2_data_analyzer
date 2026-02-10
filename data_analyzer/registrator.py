from dataclasses import dataclass
from threading import Lock
from typing import Callable, Optional, Type, override

from geometry_msgs.msg import (PoseWithCovariance, PoseWithCovarianceStamped,
                               TransformStamped)
from nav_msgs.msg import Odometry, Path
from rclpy import spin_once
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.time import Time
from recorder import Data, OrientationData, PositionData
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

logger = get_logger(__name__)


@dataclass
class NodeModel:
    node_name: str


@dataclass
class SubscriberModel(NodeModel):
    topic: str
    msg_type: Type


@dataclass
class TransformSubscriberModel(NodeModel):
    target_frame: str
    source_frame: str
    timer: float = 0.1


SUPPORTED_MSG_TYPES = {
    "nav_msgs/msg/Odometry": Odometry,
    "geometry_msgs/msg/PoseWithCovarianceStamped": PoseWithCovarianceStamped,
    "geometry_msgs/msg/PoseWithCovariance": PoseWithCovariance,
    "nav_msgs/msg/Path": Path,
}


class Subscriber(Node):
    def __init__(self, model: SubscriberModel):
        self.model = model
        super().__init__(model.node_name)
        self.subscription = self.create_subscription(
            model.msg_type, model.topic, self.run_callback, 20
        )
        self._lock = Lock()
        self._data_callbacks: list[Callable[[Data], None]] = []

    def __repr__(self):
        return self.name

    @property
    def name(self):
        return self.model.node_name

    @property
    def topic(self):
        return self.model.topic

    def run_callback(self, msg):
        raise NotImplementedError

    def add_data_callback(self, callback: Callable[[Data], None]) -> None:
        self._data_callbacks.append(callback)

    def remove_data_callback(self, callback: Callable[[Data], None]) -> None:
        if callback in self._data_callbacks:
            self._data_callbacks.remove(callback)

    def _emit_data(self, data: Data) -> None:
        for callback in self._data_callbacks:
            callback(data)


class PoseSubscriber(Subscriber):
    def __init__(self, model: SubscriberModel):
        self.x = []
        self.y = []
        super().__init__(model=model)

    def get_pose(self, msg):
        if isinstance(msg, PoseWithCovarianceStamped) or isinstance(msg, Odometry):
            return msg.pose.pose
        if isinstance(msg, PoseWithCovariance):
            return msg.pose
        if isinstance(msg, Path):
            if not msg.poses:
                return None
            return msg.poses[-1].pose  # type: ignore
        else:
            self.get_logger().error(
                "Unsupported type of message. "
                f"Should be one of: [{', '.join(SUPPORTED_MSG_TYPES.keys())}]"
            )
            return None

    def get_trajectory_data(self):
        with self._lock:
            return self.x, self.y

    def update_data(self, pose):
        with self._lock:
            x, y, _ = get_coordinates(pose)
            self.get_logger().debug(f"Coordinates: {x} {y}")
            self.x.append(x)
            self.y.append(y)

    @override
    def run_callback(self, msg):
        pose = self.get_pose(msg)
        if pose:
            self.get_logger().debug(
                f"Received {self.model.msg_type.__name__} message from topic {self.model.topic}"
            )
            self.get_logger().debug(
                f"Position: {get_position(pose)}, Orientation: {get_orientation(pose)}"
            )
            self.update_data(pose)
            self.return_data(msg, pose)

    def return_data(self, msg, pose):
        timestamp = self._extract_timestamp(msg)
        position = pose.position
        orientation = pose.orientation
        data = Data(
            timestamp=timestamp,
            position=PositionData(x=position.x, y=position.y, z=position.z),
            orientation=OrientationData(
                x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w
            ),
        )
        self._emit_data(data)

    def _extract_timestamp(self, msg) -> float:
        if isinstance(msg, Path) and msg.poses:
            stamp = msg.poses[-1].header.stamp
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9

        return float(self.get_clock().now().nanoseconds) * 1e-9


class PathSubscriber(Subscriber):
    def __init__(self, model: SubscriberModel):
        super().__init__(model=model)

    def get_path(self, msg):
        if isinstance(msg, Path):
            return msg.poses
        else:
            self.get_logger().error(f"Message is not of supported type: [{Path}]")

    @override
    def run_callback(self, msg: Odometry):
        self.get_logger().debug(
            f"Received {self.model.msg_type.__name__} message from topic {self.model.topic}"
        )

    # TODO: add path processing


def create_pose_subscriber(
    topic: str,
    msg_type: Optional[Type] = None,
    node_name: str = "",
    timeout: float = 1.0,
) -> PoseSubscriber:
    if not node_name:
        node_name = topic.replace("/", "_") + "_subscriber"

    if not msg_type:
        try:
            msg_type = get_msg_type(topic, timeout=timeout)
        except ValueError as err:
            logger.error(f"Error determining message type for topic '{topic}': {err}")
            logger.error(
                f"Setting default message type to Odometry for topic '{topic}'"
            )
            msg_type = Odometry

    if msg_type in SUPPORTED_MSG_TYPES.values():
        return PoseSubscriber(
            SubscriberModel(node_name=node_name, topic=topic, msg_type=msg_type)
        )
    raise ValueError(
        "Unsupported message type. "
        f"Should be one of [{', '.join(SUPPORTED_MSG_TYPES.keys())}]"
    )


def get_msg_type(topic: str, node: Optional[Node] = None, timeout: float = 1.0) -> Type:
    n_topic = topic if topic.startswith("/") else f"/{topic}"
    tmp_node = None
    if not node:
        tmp_node = Node("_tmp_topic_node")
        node = tmp_node

    matching_types = []
    iter_delay = 0.2  # secs between rechecks
    for _ in range(int(timeout / iter_delay)):
        spin_once(node, timeout_sec=iter_delay)

        for t_name, t_types in node.get_topic_names_and_types():
            if t_name == n_topic:
                matching_types = t_types
                break
        if matching_types:
            break

    if tmp_node:
        tmp_node.destroy_node()

    if matching_types:
        for t_type in matching_types:
            if t_type in SUPPORTED_MSG_TYPES:
                return SUPPORTED_MSG_TYPES[t_type]
    raise ValueError(
        "No supported topics found. "
        f"Should be one of {', '.join(SUPPORTED_MSG_TYPES.keys())}"
    )


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
        self.timer = self.create_timer(0.01, self.cb_timer)

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
            self.get_logger().debug(
                f"Translation: x={translation.x:.2f}, y={translation.y:.2f}, z={translation.z:.2f} | "
                f"Rotation: x={rotation.x:.2f}, y={rotation.y:.2f}, z={rotation.z:.2f}, w={rotation.w:.2f}"
            )
            self.cb_data_process(tf_data)
        except TransformException as ex:
            self.get_logger().error(
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
