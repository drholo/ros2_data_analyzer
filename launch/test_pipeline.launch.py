import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    # --- Launch configurations ---
    bagfile = LaunchConfiguration("bagfile")
    record_dst = LaunchConfiguration("record_dst")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_frame = LaunchConfiguration("map_frame", default="map")
    base_frame = LaunchConfiguration("base_frame", default="base_link")
    imu_topic = LaunchConfiguration("imu_topic", default="ouster/imu:IMU")
    path_topic = LaunchConfiguration("path_topic", default="/tf_path")

    # --- Paths to executables ---
    data_analyzer_prefix = get_package_prefix("data_analyzer")
    path_publisher_exec = Path(data_analyzer_prefix, "lib", "data_analyzer", "path_publisher")
    controller_exec = Path(data_analyzer_prefix, "lib", "data_analyzer", "controller")

    # --- Declare arguments ---
    ld = LaunchDescription([
        DeclareLaunchArgument("bagfile", default_value="", description="Path to rosbag file"),
        DeclareLaunchArgument(
            "record_dst",
            default_value=str(
                Path(get_package_share_directory("data_analyzer"), "recordings",
                     f"record_{datetime.now():%Y_%m_%d_%H_%M_%S}")
            ),
            description="Recording destination",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time"),
        DeclareLaunchArgument("map_frame", default_value="map", description="Map frame"),
        DeclareLaunchArgument("base_frame", default_value="base_link", description="Base frame"),
        DeclareLaunchArgument("imu_topic", default_value="ouster/imu:IMU", description="IMU topic"),
        DeclareLaunchArgument("path_topic", default_value="/tf_path", description="Path topic"),
    ])

    # --- Context to store PIDs and recorder status ---
    launch_context = {"path_pid": None, "amcl_pid": None, "recorder_started": False, "recorder_proc": None}

    # --- Path publisher ---
    path_publisher_proc = ExecuteProcess(
        cmd=[
            path_publisher_exec.as_posix(),
            "-b", map_frame,
            "-t", base_frame,
            "--publish_path",
            "--path_topic", path_topic,
        ],
        output="screen",
    )
    ld.add_action(path_publisher_proc)

    # --- AMCL lifecycle node ---
    amcl_node = LifecycleNode(
        namespace="",
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("scan", "/scan"), ("base_link", base_frame)],
    )
    ld.add_action(amcl_node)

    # --- AMCL display launch ---
    pkg_share = get_package_share_directory("amcl-ekf-slam")
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "display.launch.py")),
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
    ld.add_action(amcl_launch)

    # --- Function to maybe start recorder ---
    def maybe_start_recorder(context):
        if launch_context["recorder_started"]:
            return []

        if launch_context["path_pid"] and launch_context["amcl_pid"]:
            print(f"[INFO] Starting recorder for PIDs: path={launch_context['path_pid']}, amcl={launch_context['amcl_pid']}")
            recorder_proc = ExecuteProcess(
                cmd=[
                    controller_exec.as_posix(),
                    "record",
                    "tf_path:TF", "odometry/filtered:EKF", "amcl_pose:AMCL",
                    "--imu", imu_topic,
                    "--record_to", record_dst,
                    "--watchdog_ignore", path_topic,
                    "--watchdog_timeout", "5.0",
                    "--follow_pids",
                    str(launch_context["path_pid"]),
                    str(launch_context["amcl_pid"]),
                    "--plot",
                ],
                output="screen",
            )
            launch_context["recorder_proc"] = recorder_proc
            launch_context["recorder_started"] = True

            # Ensure launch shuts down if recorder dies
            return [
                recorder_proc,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=recorder_proc,
                        on_exit=[Shutdown()],
                    )
                )
            ]
        return []

    # --- Handlers to store PIDs ---
    def store_path_pid(event, context):
        launch_context["path_pid"] = event.pid
        return maybe_start_recorder(context)

    def store_amcl_pid(event, context):
        launch_context["amcl_pid"] = event.pid
        return maybe_start_recorder(context)

    # --- Event handlers for process start ---
    ld.add_action(RegisterEventHandler(OnProcessStart(target_action=path_publisher_proc, on_start=store_path_pid)))
    ld.add_action(RegisterEventHandler(OnProcessStart(target_action=amcl_node, on_start=store_amcl_pid)))

    return ld
