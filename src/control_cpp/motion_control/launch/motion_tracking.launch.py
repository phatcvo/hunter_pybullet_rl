# src/control_cpp/motion_tracking/launch/tracking_sim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pybullet_sim",
                executable="sim_node",
                name="pybullet_hunter",
                output="screen",
                parameters=[
                    {"use_gui": True},
                    {"update_rate": 100.0},
                    {"publish_lidar": True},
                ],
            ),
            Node(
                package="motion_control",
                executable="motion_control",
                name="motion_tracking",
                output="screen",
            ),
        ]
    )
