import os
from glob import glob
from setuptools import find_packages, setup

package_name = "data_analyzer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.sh'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrii Kudriashov",
    maintainer_email="andrii@kudriashov.net",
    description="ROS2 data analyzing and visualizing tool",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "controller = data_analyzer.controller:main",
            "path_publisher = data_analyzer.path_publisher:main",
        ],
    },
)
