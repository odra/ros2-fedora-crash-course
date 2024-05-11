from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pub_node = Node(
        package='sample_pkg',
        executable='simple_pub',
        name='pubsub_pub',
        parameters=[
            {'nickname': 'launcher_pub'}
        ]
    )

    sub_node = Node(
        package='sample_pkg',
        executable='simple_sub',
        name='pubsub_sub'
    )

    ld.add_action(pub_node)
    ld.add_action(sub_node)

    return ld
