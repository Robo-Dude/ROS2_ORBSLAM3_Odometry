from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                ('/imu', '/imu0')
            ]
        )
    ])
