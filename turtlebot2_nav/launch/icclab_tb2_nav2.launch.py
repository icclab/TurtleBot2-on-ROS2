from argparse import Namespace
from distutils.cmd import Command
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch_ros.substitutions import FindPackageShare
import launch
import yaml

def generate_launch_description():
    map_path = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot2_nav'),
            'maps/icclab',
            'icclab_latest_map.yaml'
        )
    )
    param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot2_nav'),
            'param',
            'nav2_params.yaml'
        )
    )
    use_namespace=LaunchConfiguration('use_namespace')
    namespace=LaunchConfiguration('namespace')

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    turtlebot2_nav_dir = get_package_share_directory('turtlebot2_nav')

    namespaced_params= ReplaceString(
        source_file=param_path, replacements={"/namespace":("",namespace)}
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_path,
            description='Full path to nav2 param file to load'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'params_file': namespaced_params}.items(),
        )

    ])
