from datetime import datetime
from pathlib import Path

from ament_index_python import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base_frame = LaunchConfiguration("base_frame")
    map_frame = LaunchConfiguration("map_frame")
    path_topic = LaunchConfiguration("path_topic")
    publish_path = LaunchConfiguration("publish_path")
    record_dst = LaunchConfiguration("record_dst")
    imu_topic = LaunchConfiguration("imu_topic")
    watchdog_ignore = LaunchConfiguration("watchdog_ignore")

    record_topics = [
        "tf_path:TF",
        "odometry/filtered:EKF",
        "amcl_pose:AMCL",
    ]
    declare_imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="imu_filtered:IMU",
        description="IMU topic to subscribe to",
    )
    declare_base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot's base frame",
    )
    declare_map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Map frame name",
    )
    declare_path_topic_arg = DeclareLaunchArgument(
        "path_topic",
        default_value="/tf_path",
        description="Path topic to publish",
    )
    declare_publish_path_arg = DeclareLaunchArgument(
        "publish_path",
        default_value="true",
        description="Enable Path publishing",
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="ouster/imu:IMU",
        description="IMU data topic",
    )
    declare_record_dst_arg = DeclareLaunchArgument(
        "record_dst",
        default_value=Path(
            get_package_share_directory("data_analyzer"),
            "recordings",
            f"record_{datetime.now():%Y_%m_%d_%H_%M_%S}",
        ).as_posix(),
        description="Destination path for rosbag recording",
    )
    declare_watchdog_ignore_arg = DeclareLaunchArgument(
        "watchdog_ignore",
        default_value=path_topic,
        description="Topics that should not ping the watchdog",
    )

    data_analyzer_prefix = get_package_prefix("data_analyzer")
    path_publisher_entrypoint = Path(
        data_analyzer_prefix, "lib", "data_analyzer", "path_publisher"
    )
    controller_entrypoint = Path(
        data_analyzer_prefix, "lib", "data_analyzer", "controller"
    )

    path_publisher_exec = ExecuteProcess(
        cmd=[
            path_publisher_entrypoint.as_posix(),
            "-b",
            map_frame,
            "-t",
            base_frame,
            "--publish_path",
            "--path_topic",
            path_topic,
        ],
        condition=IfCondition(publish_path),
        output="log",
    )

    trajectory_recorder_exec = ExecuteProcess(
        cmd=[
            controller_entrypoint.as_posix(),
            "record",
            *record_topics,
            "--imu",
            imu_topic,
            "--record_to",
            record_dst,
            "--watchdog_ignore",
            watchdog_ignore,
            "--plot",
        ],
        output="log",
    )

    ld = LaunchDescription()
    ld.add_action(declare_base_frame_arg)
    ld.add_action(declare_map_frame_arg)
    ld.add_action(declare_path_topic_arg)
    ld.add_action(declare_publish_path_arg)
    ld.add_action(declare_record_dst_arg)
    ld.add_action(declare_imu_topic_arg)
    ld.add_action(declare_watchdog_ignore_arg)

    ld.add_action(path_publisher_exec)
    ld.add_action(trajectory_recorder_exec)

    return ld
