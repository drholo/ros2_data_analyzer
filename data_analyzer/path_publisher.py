import argparse

import rclpy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from registrator import TransformSubscriber


class PathPublisher(TransformSubscriber):
    def __init__(self, base_frame="base_link", target_frame="map", publish_path=False):
        super().__init__(
            target_frame=target_frame,
            source_frame=base_frame,
            node_name="path_publisher",
        )

        self.publish_path = publish_path
        self.path_msg = Path()
        self.path_publisher = self.create_publisher(Path, "tracked_path", 10)

    def cb_data_process(self, tf_data):
        self.update_path(tf_data)
        if self.publish_path:
            self.path_publisher.publish(self.path_msg)

    def update_path(self, tf_data):
        pose = PoseStamped()
        pose.header = tf_data.header
        pose.pose.position.x = tf_data.transform.translation.x
        pose.pose.position.y = tf_data.transform.translation.y
        pose.pose.position.z = tf_data.transform.translation.z
        pose.pose.orientation = tf_data.transform.rotation

        self.path_msg.header = tf_data.header
        self.path_msg.poses.append(pose)

        self.get_logger().info(f"Path length: {len(self.path_msg.poses)}")


def main():
    parser = argparse.ArgumentParser("DataAnalyzer")
    parser.add_argument(
        "-b",
        "--base_frame",
        help=("Base frame to listen for transforms (default: 'base_link')"),
        default="base_link",
        required=False,
    )
    parser.add_argument(
        "-t",
        "--target_frame",
        help=("Target frame to listen for transforms (default: 'map')"),
        default="map",
        required=False,
    )
    parser.add_argument(
        "--publish_path",
        action="store_true",
        help="Publish the path based on the transforms",
    )
    args = parser.parse_args()

    print("Starting frame listener node...")
    rclpy.init()
    path_publisher = PathPublisher(
        args.base_frame, args.target_frame, args.publish_path
    )
    path_publisher.get_logger().info("Starting frame listener node")
    if args.publish_path:
        path_publisher.get_logger().info("Path publishing enabled")
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
