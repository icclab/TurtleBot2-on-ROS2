from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    group = GroupAction(
    actions=[

        SetRemap(src='map',dst='/map'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_launch'),
                    'launch',
                    'rtabmap.launch.py'
                ])
            ]),
            launch_arguments={
                'rtabmap_args': "--delete_db_on_start",
                'compressed':'true',
                'rgb_topic': '/camera/color/image',
                'depth_topic': '/camera/depth/image',
                'camera_info_topic': '/camera/color/camera_info',
                'visual_odometry': 'false',
                'frame_id': 'base_footprint',
                'subscribe_depth': 'true',
                'approx_sync': 'true',
                'odom_topic': '/odom',
                'qos': '2',
                'args': "-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1",
                'use_sim_time': 'false',
            }.items()    
        )

    ])

    return LaunchDescription([group])