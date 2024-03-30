from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config', default='')

    bag = LaunchConfiguration('bag', default='')
    stream_rate = LaunchConfiguration('stream_rate', default=1.0)
    delay = LaunchConfiguration('delay', default=10)

    visualize = LaunchConfiguration('visualize', default=False)
    rviz_config = PathJoinSubstitution([FindPackageShare('ov2slam'), 'rviz', 'ov2slam_visualization.rviz'])

    entities = [
        DeclareLaunchArgument(
            'config',
            default_value=config,
            description='Configuration file',
        ),
        DeclareLaunchArgument(
            'bag',
            default_value=bag,
            description='ROS2 bag file',
        ),
        DeclareLaunchArgument(
            'stream_rate',
            default_value=stream_rate,
            description='ROS2 bag play rate',
        ),
        DeclareLaunchArgument(
            'delay',
            default_value=delay,
            description='Delay of ROS2 bag play',
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value=visualize,
            description='Visualize topics using rviz2',
        ),

        Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam_node',
            arguments=[config],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(visualize)
        ),

        ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '-d', delay, '-r', stream_rate])
    ]

    return LaunchDescription(entities)
