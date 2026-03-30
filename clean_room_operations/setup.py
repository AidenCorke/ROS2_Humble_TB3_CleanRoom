from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'clean_room_operations'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AidenCorke',
    maintainer_email='aidencorke@cmail.carleton.ca',
    description='Operations and planning logic for room cleaning.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
