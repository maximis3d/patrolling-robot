from setuptools import find_packages, setup
import os

package_name = 'patrolling_robot_simulation'

def package_files(directory):
    """ Recursively collect all files under a directory. """
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

def package_models(directory, path_name):
    """Collect models preserving their original structure within the models folder in the install."""
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append((os.path.join('share', package_name, path_name, os.path.relpath(path, directory)), [os.path.join(path, filename)]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package metadata and core ROS files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Including launch files
        ('share/' + package_name + '/launch', package_files('patrolling_robot_simulation/launch')),

        ('share/' + package_name + '/worlds', package_files('worlds')),

        # Including worlds, maps, and models recursively
        ('share/' + package_name + '/maps', package_files('maps')),
    ] + package_models('models', 'models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='marstonvisuals@gmail.com',
    description='Patrolling robot simulation for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "patrolling_robot_node=patrolling_robot_simulation.patrolling_robot_node:main"
        ],
    },
)
