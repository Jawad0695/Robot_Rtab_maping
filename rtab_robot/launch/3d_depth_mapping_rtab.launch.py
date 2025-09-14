# rgbd_rtabmap.launch.py
#
# Launch RTAB-Map RGB-D SLAM for mapping/localization in simulation.
# Robot: Turtlebot3 (or custom robot with RGB-D camera + lidar)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    # Custom map save path
    database_path = '/home/jawad/bocbot_ws/src/mobile-3d-lidar-sim/my_bot/map/rtabmap.db'

    parameters = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'subscribe_rgbd': False,   # use RGB+Depth topics
        'subscribe_depth': True,
        'use_sim_time': use_sim_time,
        'qos_image': qos,
        'qos_imu': qos,
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (2D mode)

        # === Mapping parameters ===
        'RGBD/CreateOccupancyGrid': 'True',
        'Grid/FromDepth': 'True',
        'Grid/RangeMax': '5.0',
        'Grid/3D': 'True',
        'Grid/NormalsK': '20',
        'Cloud/OutputVoxelized': 'True',

        # === Save path ===
        'database_path': database_path,
    }

    remappings = [
        # RGB topics
        ('rgb/image', '/camera/image_raw'),
        ('rgb/image/compressed', '/camera/image_raw/compressed'),
        ('rgb/camera_info', '/camera/camera_info'),

        # Depth topics
        ('depth/image', '/camera/depth/image_raw'),
        ('depth/image/compressed', '/camera/depth/image_raw/compressed'),
        ('depth/camera_info', '/camera/depth/camera_info'),

        # Point cloud
        ('points', '/camera/points'),
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # === RGB-D Odometry ===
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),

        # === RTAB-Map SLAM (Mapping Mode) ===
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings),
            # 🚫 Removed arguments=['-d'] so it doesn't delete the map

        # === RTAB-Map (Localization Mode) ===
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_localization',
            output='screen',
            parameters=[parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        # === RTAB-Map Visualization ===
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])

