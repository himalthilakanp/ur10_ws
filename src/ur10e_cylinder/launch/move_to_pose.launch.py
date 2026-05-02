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
        )
    )

    # ── THIS WAS MISSING ──────────────────────────────────────────
    # Spawns arm_controller and joint_state_broadcaster so the
    # /arm_controller/follow_joint_trajectory action server comes up.
    spawn_controllers = TimerAction(
        period=5.0,   # wait for controller_manager to be ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ur10e_moveit_config"),
                        "launch",
                        "spawn_controllers.launch.py"
                    )
                )
            )
        ]
    )
    # ─────────────────────────────────────────────────────────────

    move_script = TimerAction(
        period=12.0,   # wait for demo + controllers to be fully ready
        actions=[
            Node(
                package="ur10e_cylinder",
                executable="move_to_pose",
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        moveit_demo,
        spawn_controllers,   # added
        move_script,
    ])
