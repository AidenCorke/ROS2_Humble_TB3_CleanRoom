from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'clean_room'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aiden Corke',
    maintainer_email='aidencorke@cmail.carleton.ca',
    description='Turtlebot3 robot cleaner cleaning room action server and client.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'clean_room_server = clean_room.clean_room_server:main',
            'clean_room_client = clean_room.clean_room_client:main'
        ],
    },
)
