"""Launch the live Pico transport, robot model, odometry, and LiDAR core."""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_model = PathJoinSubstitution([FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro'])
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'),' ',robot_model,' ','base_joint_type:=floating']),value_type=str)
    robot_state_publisher = Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',output='screen',parameters=[{'robot_description':robot_description,'use_sim_time':False}])
    serial_bridge = Node(package='pico_serial_bridge',executable='serial_bridge_node',name='serial_bridge_node',output='screen',parameters=[{'port':'/dev/ttyAMA0','fallback_port':'/dev/serial0','camera_tilt_position_rad':0.0}])
    differential_odometry = Node(package='pico_serial_bridge',executable='differential_odometry_node',name='differential_odometry_node',output='screen',parameters=[{'wheel_radius':0.03335,'wheel_base':0.28}])
    lidar_scan = Node(package='pico_serial_bridge',executable='lidar_scan_node',name='lidar_scan_node',output='screen',parameters=[{'frame_id':'laser_link','servo_min_angle_deg':0.0,'servo_max_angle_deg':180.0,'forward_angle_deg':90.0,'lidar_distance_scale':0.01,'range_min':0.02,'range_max':4.0,'reversal_threshold_deg':1.0,'sweep_timeout':0.25}])
    return LaunchDescription([LogInfo(msg='Starting the live AMR model, Pico bridge, differential odometry, and LiDAR scan pipeline.'),robot_state_publisher,serial_bridge,differential_odometry,lidar_scan])
