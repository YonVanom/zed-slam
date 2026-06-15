"""
zed_slam.launch.py
==================
Launches the zed_slam_node and loads all parameters from
config/localize.yaml.

Usage
-----
Default (localize with existing map):
    ros2 launch zedx_pure_pursuit zed_slam.launch.py

Override the mode at launch time without editing the yaml:
    ros2 launch zedx_pure_pursuit zed_slam.launch.py mode:=mapping
    ros2 launch zedx_pure_pursuit zed_slam.launch.py mode:=lifetime

Supported modes
---------------
  localize  – pure localization on an existing .area file  (default)
  lifetime  – lifetime mapping: extend an existing .area file
  mapping   – build a brand-new map from scratch
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# --------------------------------------------------------------------------- #
# Mode → parameter overrides                                                  #
# --------------------------------------------------------------------------- #
MODE_OVERRIDES = {
    'localize': {'initial_mapping': False, 'update_map': False, 'enable_localization_only' : True},
    'lifetime': {'initial_mapping': False, 'update_map': True, 'enable_localization_only' : False},
    'mapping':  {'initial_mapping': True,  'update_map': True, 'enable_localization_only' : False},
}


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('zed_slam')
    config_file = os.path.join(pkg_share, 'config', 'localize.yaml')

    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODE_OVERRIDES:
        raise ValueError(
            f"Unknown mode '{mode}'. "
            f"Valid options: {list(MODE_OVERRIDES.keys())}"
        )

    overrides = MODE_OVERRIDES[mode]
    overrides['publish_image'] = LaunchConfiguration('publish_image').perform(context).lower() == 'true'
    overrides['publish_pointcloud'] = LaunchConfiguration('publish_pointcloud').perform(context).lower() == 'true'
    overrides['publish_depth'] = LaunchConfiguration('publish_depth').perform(context).lower() == 'true'
    overrides['pointcloud_rate']   = float(LaunchConfiguration('pointcloud_rate').perform(context))
    overrides['pointcloud_width']  = int(LaunchConfiguration('pointcloud_width').perform(context))
    overrides['pointcloud_height'] = int(LaunchConfiguration('pointcloud_height').perform(context))
    overrides['save_pointcloud'] = LaunchConfiguration('save_pointcloud').perform(context).lower() == 'true'
    overrides['enable_2d_mode'] = LaunchConfiguration('enable_2d_mode').perform(context).lower() == 'true'

    node = Node(
        package='zed_slam',
        executable='zed_slam.py',
        name='zed_slam_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            overrides,
        ],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='localize',
            description=(
                'Operating mode: '
                '"localize" (default) | "lifetime" | "mapping"'
            ),
        ),
        DeclareLaunchArgument(
            'publish_image',
            default_value='false',
            description='Publish left rectified image on /zed/zed_node/left/image_rect_color',
        ),
        DeclareLaunchArgument(
            'publish_pointcloud',
            default_value='false',
            description='Publish point cloud on /zed/zed_node/point_cloud/cloud_registered',
        ),
        DeclareLaunchArgument(
            'publish_depth',
            default_value='false',
            description='Publish depth image on /zed/zed_node/depth/depth_registered',
        ),
        DeclareLaunchArgument(
            'pointcloud_rate',
            default_value='5.0',
            description='Point cloud publish rate in Hz (runs in its own thread)',
        ),
        DeclareLaunchArgument(
            'pointcloud_width',
            default_value='448',
            description='Point cloud retrieval width (COMPACT=448, FULL=896)',
        ),
        DeclareLaunchArgument(
            'pointcloud_height',
            default_value='256',
            description='Point cloud retrieval height (COMPACT=256, FULL=512)',
        ),
        DeclareLaunchArgument(
            'save_pointcloud',
            default_value='false',
            description='Save fused PLY pointcloud alongside area_file on shutdown/service call (mapping/lifetime modes only)',
        ),
        DeclareLaunchArgument(
            'enable_2d_mode',
            default_value='false',
            description='Constrain positional tracking to the XY ground plane (sets enable_2d_ground_mode in ZED SDK)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])