from setuptools import setup
from glob import glob

package_name = 'clean_room'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.utils',
        f'{package_name}.planner_core',
        f'{package_name}.planner_core.mapping',
        f'{package_name}.planner_core.planning',
        f'{package_name}.planner_core.visualizing',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),

        # Config files
        (f'share/{package_name}/config', glob('config/*.yaml')),

        # Map files
        (f'share/{package_name}/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aiden Corke',
    maintainer_email='aidencorke@cmail.carleton.ca',
    description='This package contains a heuristic path planner for robotic vacuums. For use in a pre-mapped environment.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clean_room = clean_room.planner_node:main',
        ],
    },
)

