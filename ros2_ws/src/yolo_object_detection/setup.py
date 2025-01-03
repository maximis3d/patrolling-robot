from setuptools import setup

package_name = 'yolo_object_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python', 'torch'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='YOLO Object Detection with ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_object_detection_node = yolo_object_detection.yolo_object_detection_node:main'
        ],
    },
)
