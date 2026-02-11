from pathlib import Path

from ament_index_python import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    record_dst = LaunchConfiguration("record_dst")

    declare_record_dst_arg = DeclareLaunchArgument(
        "record_dst",
        description="Destination path for rosbag recording",
    )

    data_analyzer_prefix = get_package_prefix("data_analyzer")
    controller_entrypoint = Path(
        data_analyzer_prefix, "lib", "data_analyzer", "controller"
    )

    trajectory_plotter_exec = ExecuteProcess(
        cmd=[
            controller_entrypoint.as_posix(),
            "plot",
            record_dst,
        ],
        output="log",
    )

    ld = LaunchDescription()
    ld.add_action(declare_record_dst_arg)
    ld.add_action(trajectory_plotter_exec)

    return ld
