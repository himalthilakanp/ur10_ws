from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur10e_cylinder", package_name="ur10e_moveit_config"
    ).to_moveit_configs()

    ld = generate_demo_launch(moveit_config)

    # Auto-spawn arm_controller at every launch
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_controller",
                "--controller-manager", "/controller_manager",
            ],
        )
    )

    return ld
