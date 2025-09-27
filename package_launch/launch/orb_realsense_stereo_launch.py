from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'enable_infra1': True,
                'enable_infra2': True
            }]
        ),
        Node(
            package='orbslam3',
            executable='stereo',
            name='orbslam3_stereo',
            output='screen',
            arguments=[
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt',
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/config/stereo/RealSense_D435i.yaml',
                'false'
            ],
            remappings=[
                ('/camera/left', '/camera/camera/infra1/image_rect_raw'),
                ('/camera/right', '/camera/camera/infra2/image_rect_raw'),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])
