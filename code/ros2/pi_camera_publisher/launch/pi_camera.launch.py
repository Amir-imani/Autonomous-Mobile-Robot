"""Launch the Raspberry Pi CSI camera authority."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_parameters = (
        get_package_share_directory('pi_camera_publisher')
        + '/config/pi_camera.yaml'
    )
    parameters_file = LaunchConfiguration('parameters_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'parameters_file',
                default_value=default_parameters,
                description='Pi camera parameter YAML file.',
            ),
            Node(
                package='pi_camera_publisher',
                executable='pi_camera_publisher_node',
                name='pi_camera_publisher',
                output='screen',
                parameters=[parameters_file],
                emulate_tty=True,
            ),
        ]
    )
