import os
from glob import glob

from setuptools import setup

package_name = "histogram_lane_filter"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "config", "histogram_lane_filter_node"),
            glob("config/histogram_lane_filter_node/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Liam Paull",
    maintainer_email="paull@mit.edu",
    description="Histogram lane filter of the lx-estimation Learning Experience (ROS 2)",
    license="GPLv3",
    entry_points={
        "console_scripts": [
            "histogram_lane_filter_node = histogram_lane_filter.histogram_lane_filter_node:main",
        ],
    },
)
