"""Run a deterministic mock-camera to CUDA-inference benchmark."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    mock_parameters = (
        get_package_share_directory('vision_testbench')
        + '/config/mock_camera.yaml'
    )
    inference_parameters = (
        get_package_share_directory('gpu_vision_inference')
        + '/config/gpu_inference.yaml'
    )

    inference_node = Node(
        package='gpu_vision_inference',
        executable='gpu_vision_inference_node',
        name='gpu_vision_inference',
        output='screen',
        parameters=[
            inference_parameters,
            {
                'model_path': LaunchConfiguration('model_path'),
                'labels_path': LaunchConfiguration('labels_path'),
                'expected_frame_rate_hz': ParameterValue(
                    LaunchConfiguration('frame_rate'), value_type=float
                ),
                'prefer_gpu_decode': True,
                'require_gpu_decode': ParameterValue(
                    LaunchConfiguration('require_gpu_decode'), value_type=bool
                ),
                'decode_warmup_width': ParameterValue(
                    LaunchConfiguration('width'), value_type=int
                ),
                'decode_warmup_height': ParameterValue(
                    LaunchConfiguration('height'), value_type=int
                ),
                'metrics_enabled': True,
                'metrics_output_path': LaunchConfiguration('metrics_output_path'),
                'metrics_format': LaunchConfiguration('metrics_format'),
                'metrics_run_id': LaunchConfiguration('run_id'),
                'metrics_append': ParameterValue(
                    LaunchConfiguration('metrics_append'), value_type=bool
                ),
                'metrics_statistics_capacity': ParameterValue(
                    LaunchConfiguration('metrics_statistics_capacity'), value_type=int
                ),
            },
        ],
        emulate_tty=True,
    )

    mock_node = Node(
        package='vision_testbench',
        executable='mock_camera_publisher_node',
        name='mock_camera_publisher',
        output='screen',
        parameters=[
            mock_parameters,
            {
                'source_type': LaunchConfiguration('source_type'),
                'video_path': LaunchConfiguration('video_path'),
                'webcam_device': LaunchConfiguration('webcam_device'),
                'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                'frame_rate': ParameterValue(
                    LaunchConfiguration('frame_rate'), value_type=int
                ),
                'max_frames': ParameterValue(
                    LaunchConfiguration('max_frames'), value_type=int
                ),
                'wait_for_subscriber': True,
            },
        ],
        emulate_tty=True,
    )

    stop_after_source = RegisterEventHandler(
        OnProcessExit(
            target_action=mock_node,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        EmitEvent(
                            event=Shutdown(
                                reason='mock stream finished; benchmark drain complete'
                            )
                        )
                    ],
                )
            ],
        )
    )
    stop_if_inference_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=inference_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='inference process exited during benchmark')
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('video_path', default_value='', description='Absolute path to the benchmark MP4 file.'),
            DeclareLaunchArgument('source_type', default_value='video_file', description='video_file, webcam, or test_pattern.'),
            DeclareLaunchArgument('webcam_device', default_value='/dev/video0', description='V4L2 device used for webcam mode.'),
            DeclareLaunchArgument('model_path', description='Absolute path to a YOLOv8 detection ONNX model.'),
            DeclareLaunchArgument('labels_path', default_value='', description='Optional newline-delimited class names file.'),
            DeclareLaunchArgument('width', default_value='1280'),
            DeclareLaunchArgument('height', default_value='720'),
            DeclareLaunchArgument('frame_rate', default_value='30'),
            DeclareLaunchArgument('max_frames', default_value='0', description='Stop after N published frames; zero runs the full file.'),
            DeclareLaunchArgument('metrics_output_path', default_value='/tmp/vision_benchmark.csv', description='CSV or JSONL per-frame telemetry output path.'),
            DeclareLaunchArgument('metrics_format', default_value='csv', description='csv or jsonl.'),
            DeclareLaunchArgument('metrics_append', default_value='false', description='Append per-frame records instead of truncating the output.'),
            DeclareLaunchArgument('metrics_statistics_capacity', default_value='1000000', description='Maximum processed frames retained for exact quantiles.'),
            DeclareLaunchArgument('run_id', default_value='baseline', description='Experiment/run identifier written into every record.'),
            DeclareLaunchArgument('require_gpu_decode', default_value='true', description='Fail rather than mix CPU JPEG decode into the benchmark.'),
            stop_after_source,
            stop_if_inference_exits,
            inference_node,
            mock_node,
        ]
    )
