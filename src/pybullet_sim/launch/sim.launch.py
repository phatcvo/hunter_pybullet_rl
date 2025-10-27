from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pybullet_sim",
                executable="sim_node",
                name="sim_node",
                output="screen",
                parameters=[
                    {
                        "use_gui": True,
                        "load_env": True,
                        "load_objects": True,
                        "publish_lidar": True,
                    }
                ],
            )
        ]
    )
