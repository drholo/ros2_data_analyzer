import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from registrator import TransformSubscriber


class PathPublisher(TransformSubscriber):
    def __init__(
        self,
        target_frame="map",
        base_frame="base_link",
        publish_path=False,
        path_topic="tracked_path",
    ):
        super().__init__(
            target_frame=target_frame,
            source_frame=base_frame,
            node_name="path_publisher",
        )

        self.publish_path = publish_path
        self.path_topic = path_topic
        self.path_msg = Path()
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)

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
    parser.add_argument(
        "--path_topic",
        help=("Topic name to publish the path (default: 'tracked_path')"),
        default="tracked_path",
        required=False,
    )
    args = parser.parse_args()

    print("Starting frame listener node...")
    rclpy.init()
    path_publisher = PathPublisher(
        args.base_frame, args.target_frame, args.publish_path, args.path_topic
    )
    path_publisher.get_logger().info("Starting frame listener node")
    if args.publish_path:
        path_publisher.get_logger().info("Path publishing enabled")
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        path_publisher.get_logger().info("Shutting down due to KeyboardInterrupt")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
