from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    config = '/ws/src/ov2slam/parameters_files/accurate/kitti/kitti_00-02.yaml'

    bag = '/ws/src/ov2slam/rosbags/kitti_2011_09_26_drive_0002_synced'
    stream_rate = 1.0
    delay = 10

    visualize = False
    rviz = '/ws/src/ov2slam/ov2slam_visualization.rviz'

    entities = [
        Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam_node',
            output='screen',
            arguments=[config],
        )
    ]

    if bag != '':
        entities.append(
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag, f'-d {delay}', f'-r {stream_rate}'],
                output='screen'
            )
        )

    if visualize:
        entities.append(
                Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz],
            )
        )

    return LaunchDescription(entities)
