from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ZhiyuanZhang_object_follower',
            namespace='nick',
            executable='find_object.py',
            name='find_object'
        ),
        Node(
            package='ZhiyuanZhang_object_follower',
            namespace='nick1',
            executable='rotate_robot.py',
            name='rotate_robot'
        ),
    ])
