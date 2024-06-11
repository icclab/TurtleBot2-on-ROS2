from launch import LaunchDescription

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', 's mcap', '-d 5', '-o my_bag'],
        )
    ])
