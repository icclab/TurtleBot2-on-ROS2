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
            'map',
            'lab07.yaml'
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
    rviz_config_file = LaunchConfiguration('rviz_config')

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    turtlebot2_nav_dir = get_package_share_directory('turtlebot2_nav')

    turtlebot2_description_dir = FindPackageShare(package='turtlebot2_description').find('turtlebot2_description')

    kobuki_node_dir = get_package_share_directory('kobuki_node')
    params_file = os.path.join(kobuki_node_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    urg_node_dir = get_package_share_directory('urg_node')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

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
            default_value='tb2_6',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),

        DeclareLaunchArgument(  # TODO! automatic substitution namespace for proper topics
            'rviz_config',
            default_value=os.path.join(
                turtlebot2_nav_dir, 
                'rviz', 
                'namespaced_tb2_6.rviz'),
            description='Full path to the RVIZ config file to use'),
        
        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the navigation stack'),
        
        Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            namespace=namespace,
            parameters=[params],
            remappings=[('commands/velocity','cmd_vel'),('/tf','tf'),('/tf_static','tf_static')]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            parameters=[{'robot_description': launch.substitutions.Command(['xacro ',os.path.join(turtlebot2_description_dir,'robots/kobuki_hexagons_hokuyo.urdf.xacro')])}]
        ),
        
        Node(
           package='urg_node',
           executable='urg_node_driver',
           output='screen',
           namespace=namespace,
           parameters=[{"ip_adress":"192.168.0.10"},{"ip_port":10940},{"serial_baud":115200}],
           remappings=[('/tf','tf'),('/tf_static','tf_static'),('/scan','scan')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'params_file': param_path}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen',
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/goal_pose', 'goal_pose'),
                        ('/clicked_point', 'clicked_point'),
                        ('/initialpose', 'initialpose')]),  
    ])