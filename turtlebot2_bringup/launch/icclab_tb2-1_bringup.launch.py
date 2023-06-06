import os
from struct import pack

from setuptools import Command

from ament_index_python import get_package_share_directory, get_package_share_path

import launch
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions

def generate_launch_description():
    kobuki_package = launch_ros.substitutions.FindPackageShare(package='kobuki_node').find('kobuki_node')
    lidar_package = launch_ros.substitutions.FindPackageShare(package='sllidar_ros2').find('sllidar_ros2')
    turtlebot2_bringup_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_bringup').find('turtlebot2_bringup')
    turtlebot_description_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_description').find('turtlebot2_description')

    ekf_config_params = os.path.join(turtlebot2_bringup_package,'config/ekf_config.yaml')

    kobuki_node_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                kobuki_package,
                'launch/kobuki_node-launch.py')
        )
    )

    ekf_node = launch_ros.actions.Node(
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
