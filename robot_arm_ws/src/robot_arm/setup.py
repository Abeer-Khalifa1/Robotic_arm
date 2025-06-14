from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.1',  # Updated from 0.0.0
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Add URDF/SDF models if needed
        (os.path.join('share', package_name, 'models'),
         glob('models/*.sdf')),
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # Updated from TODO
    maintainer_email='your@email.com',  # Updated from TODO
    description='ROS 2 package for robotic arm simulation',  # Updated from TODO
    license='Apache-2.0',  # Updated from TODO
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_ik_node = robot_arm.square_ik_node:main',
    ],
},
)
