from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ZhiyuanZhang_object_follower',
            namespace='nick',
            executable='find_object',
            name='find_object'
        ),
        Node(
            package='ZhiyuanZhang_object_follower',
            namespace='nick1',
            executable='rotate_robot',
            name='rotate_robot'
        ),
        Node(
            package='ZhiyuanZhang_object_follower',
            namespace='nick1',
            executable='view_debug_img',
            name='view_debug_img'
        ),
        Node(
            package='ZhiyuanZhang_object_follower',
            namespace='nick1',
            executable='send_test_img',
            name='send_test_img'
        ),
    ])
