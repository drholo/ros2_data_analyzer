import math

from geometry_msgs.msg import Pose

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):
    pass


def main():
    print("Starting frame listener node...")
    rclpy.init()
    node = FrameListener()
    node.get_logger().info('Starting frame listener node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()