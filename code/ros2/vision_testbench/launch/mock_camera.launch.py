"""Launch the hardware-agnostic compressed camera source."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_parameters = (
        get_package_share_directory('vision_testbench')
        + '/config/mock_camera.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('parameters_file', default_value=default_parameters, description='Mock camera parameter YAML file.'),
            DeclareLaunchArgument('source_type', default_value='video_file', description='video_file, webcam, or test_pattern.'),
            DeclareLaunchArgument('video_path', default_value='', description='Absolute path to the benchmark video.'),
            DeclareLaunchArgument('webcam_device', default_value='/dev/video0', description='V4L2 device used for webcam mode.'),
            DeclareLaunchArgument('max_frames', default_value='0', description='Stop after this many published frames; zero means all.'),
            Node(
                package='vision_testbench',
                executable='mock_camera_publisher_node',
                name='mock_camera_publisher',
                output='screen',
                parameters=[
                    LaunchConfiguration('parameters_file'),
                    {
                        'source_type': LaunchConfiguration('source_type'),
                        'video_path': LaunchConfiguration('video_path'),
                        'webcam_device': LaunchConfiguration('webcam_device'),
                        'max_frames': ParameterValue(LaunchConfiguration('max_frames'), value_type=int),
                    },
                ],
                emulate_tty=True,
            ),
        ]
    )
