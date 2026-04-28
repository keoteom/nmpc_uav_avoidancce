from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters = [{
        'frame_id': 'base_link',  # 드론 기준
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,      # ← PointCloud 입력!
        'approx_sync': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_imu': qos,                    # IMU 필요시
        'Reg/Strategy': '1',               # ICP
        'Reg/Force3DoF': 'True',           # 드론이면 True 권장
        'Grid/RangeMin': '0.2',
        'Optimizer/GravitySigma': '0'      # 2D constraint
    }]

    remappings = [
        ('scan_cloud', '/depth_camera/points'),   # ✅ 포인트클라우드 입력
        ('odom', '/fmu/out/vehicle_odometry')     # ✅ PX4 오도메트리
    ]

    return LaunchDescription([

        # -------------------- Launch Arguments --------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'qos',
            default_value='2',
            description='QoS for input topics'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Localization mode'
        ),

        # -------------------- SLAM Mode --------------------
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
        ),

        # -------------------- Localization Mode --------------------
        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros',
            executable='rtabmap',
            output='screen',
            parameters=[
                *parameters,
                {'Mem/IncrementalMemory': 'False',
                 'Mem/InitWMWithAllNodes': 'True'}
            ],
            remappings=remappings
        ),

        # -------------------- Visualization --------------------
        Node(
            package='rtabmap_ros',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        )
    ])
