import os
from struct import pack

from setuptools import Command

from ament_index_python import get_package_share_directory, get_package_share_path
from launch_ros.actions import Node
import launch
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions
import yaml

def generate_launch_description():
    kobuki_package = get_package_share_directory('kobuki_node')
    lidar_package = get_package_share_directory('sllidar_ros2')
    turtlebot2_bringup_package = get_package_share_directory('turtlebot2_bringup')
    turtlebot_description_package = get_package_share_directory('turtlebot2_description')

    ekf_config_params = os.path.join(turtlebot2_bringup_package,'config/ekf_config.yaml')

    params_file = os.path.join(turtlebot2_bringup_package, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_node_launch =  Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            #namespace=namespace,
            parameters=[params],
            remappings=[('commands/velocity','cmd_vel'),('/tf','tf'),('/tf_static','tf_static')]
        )

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            parameters=[ekf_config_params],
            remappings=[("odometry/filtered", "odom")]
        )

    lidar_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                lidar_package,
                'launch/sllidar_s1_launch.py'
            )
        ),
        launch_arguments= {'serial_port':'/dev/lidar',
                           'frame_id': 'base_scan'}.items()
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ',os.path.join(turtlebot_description_package,'robots/icclab_tb2-1_urdf.xacro')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',os.path.join(turtlebot2_bringup_package,'rviz/bringup.rviz')],
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("open_rviz")) 
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),
        kobuki_node_launch,
        lidar_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # rviz_node,
        # ekf_node
    ])

