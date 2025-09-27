from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='stereo-inertial',
            name='orbslam3_stereo_inertial',
            output='screen',
            arguments=[
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt',
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/config/stereo-inertial/RealSense_D445.yaml',
                'false'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])
