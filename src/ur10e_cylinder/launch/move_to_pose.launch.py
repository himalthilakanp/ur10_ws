from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur10e_moveit_config"),
                "launch",
                "demo.launch.py"
            )
        ),
        launch_arguments={
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner"
        }.items(),
    )

    move_script = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="ur10e_cylinder",
                executable="move_to_pose",
                output="screen",
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        moveit_demo,
        move_script,
    ])