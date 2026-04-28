import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Global sim time
        SetParameter(name='use_sim_time', value=use_sim_time),
        
        # ros_gz_bridge /clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # px4_ros_com tf2_control
        Node(
            package='px4_ros_com',
            executable='tf2_control',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Static TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),
        
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ])
