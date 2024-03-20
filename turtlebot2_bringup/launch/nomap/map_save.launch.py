from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    ld = LaunchDescription()
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_cli",
    )
 
    ld.add_action(map_saver)
    return ld

