from glob import glob

from setuptools import find_packages, setup

package_name = "kinect_v2_ros2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Christian Dörflinger",
    maintainer_email="michdo93@users.noreply.github.com",
    description="ROS 2 driver for the Kinect for Xbox One",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["kinect_v2_node = kinect_v2_ros2.node:main"]},
)
