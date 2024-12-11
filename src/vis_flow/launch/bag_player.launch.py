from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bag_file_path = '/workspaces/perception_ws/data/rosbag2_2024_12_01-15_19_46'  # Update with your bag file path

    return LaunchDescription([
        Node(
            package='rosbag2',
            executable='play',
            name='rosbag2_play',
            output='screen',
            arguments=[bag_file_path]
        ),
        Node(
            package='vis_flow',
            executable='player',
            name='player',
            output='screen'
        )
    ])