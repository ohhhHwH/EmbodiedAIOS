from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_path_fixer',
            executable='fix_path_header',
            name='fix_path_header',
            output='screen',
        ),
        Node(
            package='goal_path_fixer',
            executable='goal_pose_refresher',
            name='goal_pose_refresher',
            output='screen',
        ),
    ])
