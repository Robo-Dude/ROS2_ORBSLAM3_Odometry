from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_launch',
            executable='cam2img',
            name='cam2img',
            output='screen',
        ),
        Node(
            package='orbslam3',
            executable='mono',
            name='mono',
            output='screen',
            arguments=[
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt',
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/config/monocular/EuRoC.yaml',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])
