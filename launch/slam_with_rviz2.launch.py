#!/usr/bin/env python3
"""
Launch file for ORB-SLAM3 with RViz2 Visualization
Starts camera, SLAM, bridge, and RViz2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='4',
        description='Camera device ID'
    )
    
    settings_name_arg = DeclareLaunchArgument(
        'settings_name',
        default_value='Webcam_Sensitive',
        description='ORB-SLAM3 settings file name'
    )
    
    frame_delay_arg = DeclareLaunchArgument(
        'frame_delay',
        default_value='0.1',
        description='Delay between frame sends (seconds)'
    )
    
    # Camera node
    camera_node = Node(
        package='ros2_orb_slam3',
        executable='space_bar.py',
        name='live_mono_camera',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'settings_name': LaunchConfiguration('settings_name'),
            'frame_delay': LaunchConfiguration('frame_delay'),
            'fps': 20,
            'width': 640,
            'height': 480,
            'show_camera': True
        }]
    )
    
    # ORB-SLAM3 C++ node
    slam_node = Node(
        package='ros2_orb_slam3',
        executable='mono_node_cpp',
        name='mono_slam_cpp',
        output='screen',
        parameters=[{
            'node_name_arg': 'mono_slam_cpp'
        }]
    )
    
    # SLAM to RViz2 bridge
    bridge_node = Node(
        package='ros2_orb_slam3',
        executable='slam_to_rviz2.py',
        name='slam_to_rviz2_publisher',
        output='screen'
    )
    
    # RViz2 with custom config
    # Update this path to where you save the .rviz file
    rviz_config_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_test',
        'src',
        'ros2_orb_slam3',
        'config',
        'rviz2_orb_slam3.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        camera_id_arg,
        settings_name_arg,
        frame_delay_arg,
        slam_node,
        camera_node,
        bridge_node,
        rviz_node
    ])
