# SPDX-License-Identifier: MIT
# Copyright (c) 2025 whetfox
from setuptools import setup
import os
from glob import glob

package_name = "piper_raspi_hc_sr04"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheatfox",
    maintainer_email="wheatfox17@icloud.com",
    description="HC-SR04 ultrasonic sensor driver for Raspberry Pi",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hc_sr04_node = piper_raspi_hc_sr04.hc_sr04_ros2_node:main",
        ],
    },
)
