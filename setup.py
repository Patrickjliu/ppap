import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rosmav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ms.ibnseddik@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros_bluerov2_interface = rosmav.ros_bluerov2_interface:main",
            "ppap = rosmav.ppap:main",
            "test_dance = rosmav.test_dance:main",
            "bluerov2_camera_interface = rosmav.bluerov2_camera_interface:main",
            "depth_control = rosmav.depth_control:main",
            "heading_control = rosmav.heading_control:main",
            "arm = rosmav.arm:main",
            "lane_following = rosmav.lane_following:main",
            "desired_depth = rosmav.desired_depth:main",
            "pressure_to_depth = rosmav.pressure_to_depth:main",
        ],
    },
)
