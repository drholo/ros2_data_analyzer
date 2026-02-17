import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    odometry_topic = LaunchConfiguration("odometry_topic", default="odometry/filtered")
    bagfile = LaunchConfiguration("bagfile")
    record_dst = LaunchConfiguration("record_dst")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_frame = LaunchConfiguration("map_frame", default="map")
    base_frame = LaunchConfiguration("base_frame", default="base_link")
    imu_topic = LaunchConfiguration("imu_topic", default="ouster/imu")
    path_topic = LaunchConfiguration("path_topic", default="/tf_path")
    watchdog_timeout = LaunchConfiguration("watchdog_timeout", default="5.0")

    data_analyzer_prefix = get_package_prefix("data_analyzer")
    path_publisher_exec = Path(
        data_analyzer_prefix, "lib", "data_analyzer", "path_publisher"
    )
    controller_exec = Path(data_analyzer_prefix, "lib", "data_analyzer", "controller")

    declare_bagfile_arg = DeclareLaunchArgument(
        "bagfile", default_value="", description="Path to rosbag file"
    )
    declare_record_dst_arg = DeclareLaunchArgument(
        "record_dst",
        default_value=str(
            Path(
                get_package_share_directory("data_analyzer"),
                "recordings",
                f"record_{datetime.now():%Y_%m_%d_%H_%M_%S}",
            )
        ),
        description="Recording destination",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    declare_map_frame_arg = DeclareLaunchArgument(
        "map_frame", default_value="map", description="Map frame name"
    )
    declare_base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value="base_link", description="Base frame name"
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="ouster/imu",
        description="IMU topic to subscribe to",
    )
    declare_path_topic_arg = DeclareLaunchArgument(
        "path_topic", default_value="/tf_path", description="Path topic to publish"
    )
    declare_watchdog_ignore_arg = DeclareLaunchArgument(
        "watchdog_ignore",
        default_value=path_topic,
        description="Topic to ignore for watchdog monitoring",
    )
    declare_watchdog_timeout_arg = DeclareLaunchArgument(
        "watchdog_timeout",
        default_value="5.0",
        description="Watchdog timeout in seconds",
    )

    ld = LaunchDescription()
    ld.add_action(declare_bagfile_arg)
    ld.add_action(declare_record_dst_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_map_frame_arg)
    ld.add_action(declare_base_frame_arg)
    ld.add_action(declare_imu_topic_arg)
    ld.add_action(declare_path_topic_arg)
    ld.add_action(declare_watchdog_ignore_arg)
    ld.add_action(declare_watchdog_timeout_arg)

    launch_context = {
        "path_pid": None,
        "recorder_started": False,
        "recorder_proc": None,
    }

    path_publisher_proc = ExecuteProcess(
        cmd=[
            path_publisher_exec.as_posix(),
            "-b",
            map_frame,
            "-t",
            base_frame,
            "--publish_path",
            "--path_topic",
            path_topic,
        ],
        output="screen",
    )
    ld.add_action(path_publisher_proc)

    pkg_share = get_package_share_directory("amcl-ekf-slam")
    amcl_ekf_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "display.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "bagfile": bagfile,
            "pointcloud_topic": "ouster/points",
            "imu_topic": imu_topic,
            "scan_topic": "/scan",
            "laser_frame": "os_sensor",
            "imu_frame": "os_imu",
            "3d_map": "true",
        }.items(),
    )
    ld.add_action(amcl_ekf_slam_launch)

    map_saver_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "--mode",
            "raw",
            "-f",
            PathJoinSubstitution([record_dst, "raw_map"]),
        ],
        output="screen",
    )

    def _start_recorder(context):
        if launch_context["recorder_started"]:
            return []

        if launch_context["path_pid"]:
            recorder_proc = ExecuteProcess(
                cmd=[
                    controller_exec.as_posix(),
                    "record",
                    "tf_path:TF",
                    "odometry/filtered:EKF",
                    "amcl_pose:AMCL",
                    "--imu",
                    imu_topic,
                    "--record_to",
                    record_dst,
                    "--watchdog_ignore",
                    path_topic,
                    odometry_topic,
                    "--watchdog_timeout",
                    watchdog_timeout,
                    "--follow_pids",
                    str(launch_context["path_pid"]),
                    "--plot",
                ],
                output="screen",
            )
            launch_context["recorder_proc"] = recorder_proc
            launch_context["recorder_started"] = True

            return [
                recorder_proc,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=recorder_proc,
                        on_exit=[
                            map_saver_proc,
                            RegisterEventHandler(
                                OnProcessExit(
                                    target_action=map_saver_proc,
                                    on_exit=[Shutdown()],
                                )
                            ),
                        ],
                    )
                ),
            ]
        return []

    def store_path_pid(event, context):
        launch_context["path_pid"] = event.pid
        return _start_recorder(context)

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(target_action=path_publisher_proc, on_start=store_path_pid)
        )
    )

    return ld
