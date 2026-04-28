#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ros_gz_bridge parameter_bridge는 arguments에 여러 브리지 규칙을 한 번에 넣어도 됩니다.
    gz_bridges = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),


        # Gazebo <-> ROS2 bridge (/clock, pointclouds)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=gz_bridges,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # px4_ros_com tf2_control
        Node(
            package='px4_ros_com',
            executable='tf2_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Static TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # 안정적으로(배포판 차이 덜 타게) flag 형식 사용
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            output='screen'
        ),

        # Static TF: base_link -> x500_lidar_2d_0/link/lidar3d
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'x500_lidar_2d_0/link/lidar3d',
            ],
            output='screen'
        ),

        # Static TF: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
