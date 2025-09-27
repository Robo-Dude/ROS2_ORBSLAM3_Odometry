from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
        ),
        Node(
            package='orbslam3',
            executable='rgbd',
            name='rgbd',
            output='screen',
            arguments=[
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt',
                '/home/robo-dude/projects/ros2_ws/src/orbslam3_ros2/config/rgb-d/RealSense_D435i.yaml',
                'false'
            ],
            remappings=[
                ('/camera/rgb', '/camera/camera/color/image_raw'),
                ('/camera/depth', '/camera/camera/depth/image_rect_raw'),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])
