import argparse

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class FrameListener(Node):
    def __init__(self, base_frame='base_link', target_frame='map'):
        super().__init__('TF_frame_listener')
        
        self.target_frame = target_frame
        self.source_frame = base_frame
        self.current_time = Time()


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.cb_timer)

    
    def cb_timer(self):
        from_frame = self.source_frame
        to_frame = self.target_frame

        try:
            tf_data = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                self.current_time,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame} to {from_frame}: {ex}')
            return
        
        translation = tf_data.transform.translation
        rotation = tf_data.transform.rotation
        self.get_logger().info(
            f'Translation: x={translation.x:.2f}, y={translation.y:.2f}, z={translation.z:.2f} | '
            f'Rotation: x={rotation.x:.2f}, y={rotation.y:.2f}, z={rotation.z:.2f}, w={rotation.w:.2f}'
        )



def main():
    parser = argparse.ArgumentParser("DataAnalyzer")
    parser.add_argument(
        "-b",
        "--base_frame",
        help=(
            "Base frame to listen for transforms (default: 'base_link')"
        ),
        default='base_link',
        required=False
    )
    parser.add_argument(
        "-t",
        "--target_frame",
        help=(
            "Target frame to listen for transforms (default: 'map')"
        ),
        default='map',
        required=False
    )
    args = parser.parse_args()


    print("Starting frame listener node...")
    rclpy.init()
    node = FrameListener(args.base_frame, args.target_frame)
    node.get_logger().info('Starting frame listener node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()