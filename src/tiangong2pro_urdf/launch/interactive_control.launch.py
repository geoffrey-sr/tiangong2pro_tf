import launch
from launch.substitutions import (
    Command,
    LaunchConfiguration
)
import launch_ros
import os
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="tiangong2pro_urdf"
    ).find("tiangong2pro_urdf")
    
    # Using the with_hands URDF as requested
    default_model_path = os.path.join(pkg_share, "urdf", "tiangong2.0_pro_with_hands.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "config", "interactive.rviz")
    prefix_script_path = os.path.join(pkg_share, "scripts", "add_urdf_prefix.py")

    
    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot URDF file",
        )
    )

    robot_description_content = Command(
        [
            "cat ",
            LaunchConfiguration("model"),
        ]
    )
    
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # 1. Real Robot State Publisher (listening to /joint_states)
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description_param],
    )

    # 2. Real Robot Status Bridge (Motor IDs -> Joint States)
    real_status_bridge_node = launch_ros.actions.Node(
        package="tiangong2pro_urdf",
        executable="joint_state_publisher.py",
        name="real_status_bridge",
        output="screen"
    )

    # 3. Interactive GUI
    interactive_gui_node = launch_ros.actions.Node(
        package="tiangong2pro_urdf",
        executable="interactive_gui.py",
        name="interactive_gui",
        output="screen"
    )

    # 4. RViz
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
    )

    nodes = [
        robot_state_publisher_node,
        real_status_bridge_node,
        interactive_gui_node,
        rviz_node,
    ]

    return launch.LaunchDescription(args + nodes)
