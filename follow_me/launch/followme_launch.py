from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    turtle1_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle2 = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    follower_node = Node(
            package='follow_me',
            executable='listener',
            name='listener',
        )
    return LaunchDescription([
        turtle1_node,
        spawn_turtle2,
        follower_node,
    ])
