import os
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('get_cube_pose'), 'rviz_config', 'launch_part.rviz')

    #add launch argument for point cloud topic


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="False", 
            description='Turn on/off sim time setting'
        ),

        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/wrist_rgbd_depth_sensor/points', 
            description='PointCloud2 topic for \'simple_grasp\' pkg to get info from.'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
            arguments=['-d', rviz_config_dir]),

        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='basic_grasping_perception_node',
            output='screen',
            parameters=[
                {'debug_topics': True},
                {'pointcloud_topic': LaunchConfiguration('pointcloud_topic')},
                ],
        ),

        Node(
            name='action_client_node',
            package='get_cube_pose',  
            executable='get_pose_client',  
            output='screen',
            parameters=[
                {'debug_topics': True},
                {'pointcloud_topic': LaunchConfiguration('pointcloud_topic')},
            ],
        ),
    ])
