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
    urg_package = launch_ros.substitutions.FindPackageShare(package='urg_node').find('urg_node')
    turtlebot2_description_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_description').find('turtlebot2_description')
    turtlebot2_slam_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_slam').find('turtlebot2_slam')
    turtlebot2_bringup_package = launch_ros.substitutions.FindPackageShare(package='turtlebot2_bringup').find('turtlebot2_bringup')
    slam_toolbox_package = launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')

    
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
            parameters=[ekf_config_params]
            # ,
            # remappings=[("odometry/filtered", "odom")]
        )
    urg_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                urg_package,
                'launch/urg_node_launch.py'
            )
        ),
        launch_arguments= {'sensor_interface':'ethernet'}.items()
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ',os.path.join(turtlebot2_description_package,'robots/kobuki_hexagons_hokuyo.urdf.xacro')])}]
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
        arguments=['-d',os.path.join(turtlebot2_slam_package,'rviz/mapping.rviz')]
    )
    
    mapping_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_package,
                'launch/online_async_launch.py')
        )
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),

        kobuki_node_launch,
        urg_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        mapping_launch,
        ekf_node
    ])
