from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="False", 
            description='\'True\' if using simulation, else \'False\''
        ),

        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='basic_grasping_perception_node',
            output='screen',
            parameters=[{'debug_topics': True}],
        ),

        Node(
            name="pick_and_place_perception",
            package="moveit2_scripts",
            executable="pick_and_place_perception",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {'use_sim_time': LaunchConfiguration("use_sim_time")},
                ],
            )
    ])