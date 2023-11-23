from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description ():
    ld = LaunchDescription()

    talker_node = Node (
        package="turtlesim",
        executable="turtlesim_node"
    )


    listener_node = Node (
        package="assg_2",
        executable="spawner"
    )

    ld.add_action(talker_node)
    ld.add_action (listener_node)

    return ld