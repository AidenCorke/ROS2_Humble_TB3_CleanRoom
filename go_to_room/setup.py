from setuptools import find_packages, setup
from glob import glob

package_name = "go_to_room"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            [f"resource/{package_name}"]),
        (f"share/{package_name}",
            ["package.xml"]),
        (f"share/{package_name}/launch",
            glob("launch/*.py")),
        (f"share/{package_name}/config",
            glob("config/*.yaml") + glob("config/*.pgm") + glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="VincentLeonard",
    maintainer_email="vleon039@uottawa.ca",
    description="Room-to-room navigation for the MCG5138 cleaning robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Main navigation server — run this first
            "go_to_room_node=go_to_room.go_to_room_node:main",
            # Action client — used from a second terminal to send goals
            "go_to_room_client=go_to_room.go_to_room_client:main",
        ],
    },
)
