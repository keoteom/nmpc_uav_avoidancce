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
        
        # Static TF: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_camera_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_link']
        ),
        
        # ros_gz_bridge /depth_camera/points
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='points_bridge',
            output='screen',
            arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked']
        ),
        
        # Octomap Server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[('/cloud_in', '/depth_camera/points')],
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.10,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_model.max_range': 5.0,
                'filter_speckles': True,
                'outrem_radius': 0.5,
                'outrem_neighbors': 5
            }]
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
    ])
