from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'package_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'orb_realsense_stereo_launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'orb_realsense_stereo_inertial_launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'orb_realsense_rgbd_launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'orb_realsense_mono_launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo-dude',
    maintainer_email='robo-dude@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cam2img = package_launch.image_pub:main"
        ],
    },
)
