import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    map_dir = os.path.join(get_package_share_directory('my_bot'), 'map')
    config_dir = os.path.join(get_package_share_directory('my_bot'), 'config')
    urdf_dir = os.path.join(get_package_share_directory('my_bot'), 'urdf')

    map_file = os.path.join(map_dir, 'my_map_save.yaml')
    param_file = os.path.join(config_dir, 'nav2_params.yaml')
    rviz_config_dir = os.path.join(urdf_dir, 'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('point_cloud_perception'),
                    'launch',
                    '3d_depth_mapping_rtab.launch.py'
                )
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('my_bot'),
                    'launch',
                    'launch_sim.launch.py'
                )
            )
        ),
    ])

