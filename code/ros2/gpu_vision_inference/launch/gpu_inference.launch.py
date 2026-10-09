"""Launch the laptop GPU perception accelerator."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_parameters = (
        get_package_share_directory('gpu_vision_inference')
        + '/config/gpu_inference.yaml'
    )
    parameters_file = LaunchConfiguration('parameters_file')
    model_path = LaunchConfiguration('model_path')
    labels_path = LaunchConfiguration('labels_path')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'parameters_file',
                default_value=default_parameters,
                description='GPU inference parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'model_path',
                description='Absolute path to the YOLO detection ONNX model.',
            ),
            DeclareLaunchArgument(
                'labels_path',
                default_value='',
                description='Optional newline-delimited class names file.',
            ),
            Node(
                package='gpu_vision_inference',
                executable='gpu_vision_inference_node',
                name='gpu_vision_inference',
                output='screen',
                parameters=[
                    parameters_file,
                    {
                        'model_path': model_path,
                        'labels_path': labels_path,
                    },
                ],
                emulate_tty=True,
            ),
        ]
    )
