from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'om_custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shantam',
    maintainer_email='shantam.s@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'move_arm = om_custom.control_om:main',
            'initial_pose = om_custom.initial_pose:main',
            'move_turtlebot_server = om_custom.move_turtlebot_action_server:main',
            'rotate_turtlebot_server = om_custom.rotate_turtlebot_action_server:main',
            'move_turtlebot_client = om_custom.move_turtlebot_action_client:main',
            'rotate_turtlebot_client = om_custom.rotate_turtlebot_action_client:main',
            'path_planning = om_custom.path_planning:main',
            'move_turtlebot_node = om_custom.move_turtlebot_node:main',
            'rotate_turtlebot_node = om_custom.rotate_turtlebot_node:main',
            'path_planning2 = om_custom.path_planning2:main',
        ],
    },
)
