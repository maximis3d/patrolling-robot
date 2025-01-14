# yolo_object_detection/setup.py
from setuptools import setup
import os
from glob import glob

package_name = "yolo_object_detection"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("yolo_object_detection/launch/*.py"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Max Marston",
    maintainer_email="marstonvisuals@gmail.com",
    description="YOLO object detection package for patrolling robot",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_object_detection_node=yolo_object_detection.yolo_object_detection_node:main",
            "yolo_image_saver_node=yolo_object_detection.yolo_image_saver_node:main",
            "baseline_manger_node=yolo_object_detection.baseline_manager_node:main",
            "patrolling_yolo_node=yolo_object_detection.patrolling_yolo_node:main",
            "state_manager_node=yolo_object_detection.state_manager_node:main"
        ],
    },
)
