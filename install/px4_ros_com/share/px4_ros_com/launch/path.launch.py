import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Global sim time
        SetParameter(name='use_sim_time', value=use_sim_time),

        # ── 기존 노드 ────────────────────────────────────────────────

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

        # ── 추가 노드 ────────────────────────────────────────────────

        # 1. RRT* planner
        Node(
            package='px4_ros_com',
            executable='rrt_star_planner',
            name='rrt_star_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_enu': True,
                'waypoint_spacing': 0.02,
            }]
        ),

        # 2. Obstacle odom publisher
        Node(
            package='px4_ros_com',
            executable='obstacle_odom_publisher',
            name='obstacle_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 3. Global path
        Node(
            package='px4_ros_com',
            executable='global_path',
            name='global_path',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 4. Odom publisher (ENU)
        Node(
            package='nmpc_uav_avoidance',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_enu': True,
            }]
        ),

        # 5. MicroXRCEAgent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888', '-d', '17'],
            name='micro_xrce_agent',
            output='screen'
        ),  

        # 6. Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'frame_id': 'map',
                'octomap_path': os.path.expanduser('~/Downloads/citymap.bt'),
            }]
        ),
    ])