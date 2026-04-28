from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = True

    return LaunchDescription([

        # 2️⃣ Gazebo → ROS lidar bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),

        # 3️⃣ Micro XRCE Agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        ),

        # 5️⃣ map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 7️⃣ px4 tf2 control
        Node(
            package='px4_ros_com',
            executable='tf2_control',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 8️⃣ base_link → lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'base_link',
                'x500_lidar_2d_0/link/lidar3d'
            ]
        ),

        # 9️⃣ Gazebo clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        ),

        # 🔟 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 1️⃣1️⃣ Octomap
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'resolution': 0.10},
                {'frame_id': 'map'},
                {'base_frame_id': 'ground'},
                {'sensor_model.max_range': 5.0},
                {'filter_ground_plane': True},
                {'ground_filter.distance': 0.05},
                {'ground_filter.angle': 0.15},
                {'ground_filter.plane_distance': 0.4},
                {'filter_speckles': True},
                {'outrem_radius': 0.5},
                {'outrem_neighbors': 5},
            ],
            remappings=[
                ('cloud_in', '/lidar/points')
            ],
            output='screen'
        ),

        # 1️⃣2️⃣ ground odom
        Node(
            package='px4_ros_com',
            executable='ground_odom',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'fixed_z': -0.47}
            ],
            output='screen'
        ),
    ])
