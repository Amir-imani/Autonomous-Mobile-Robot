"""Display the AMR description with interactive articulated joints in RViz2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the robot-description visualization launch description."""
    package_share = FindPackageShare("amr_description")
    default_model = PathJoinSubstitution([package_share, "urdf", "amr.urdf.xacro"])
    default_rviz_config = PathJoinSubstitution([package_share, "rviz", "amr_description.rviz"])
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"),value_type=bool)
    robot_description = ParameterValue(Command([FindExecutable(name="xacro"), " ", model]),value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("model",default_value=default_model,description="Absolute path to the primary AMR Xacro file."),
        DeclareLaunchArgument("rviz_config",default_value=default_rviz_config,description="Absolute path to the RViz2 configuration file."),
        DeclareLaunchArgument("use_sim_time",default_value="false",description="Use the simulation clock when set to true."),
        LogInfo(msg=["Loading AMR robot description from: ", model]),
        Node(package="robot_state_publisher",executable="robot_state_publisher",name="robot_state_publisher",output="screen",parameters=[{"robot_description":robot_description,"use_sim_time":use_sim_time}]),
        Node(package="joint_state_publisher_gui",executable="joint_state_publisher_gui",name="joint_state_publisher_gui",output="screen",parameters=[{"use_sim_time":use_sim_time}]),
        Node(package="rviz2",executable="rviz2",name="rviz2",output="screen",arguments=["-d",rviz_config],parameters=[{"use_sim_time":use_sim_time}]),
    ])
