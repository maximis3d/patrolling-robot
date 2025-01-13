from setuptools import find_packages, setup
from glob import glob

package_name = 'patrolling_robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ensure the package is properly indexed
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        ('share/' + package_name + '/launch', glob('patrolling_robot_simulation/launch/*.py')),
        
        # Worlds directory (make sure it is inside the package directory)
        ('share/' + package_name + '/worlds', glob('patrolling_robot_simulation/worlds/*.world')),
        
        # Maps directory (make sure it is inside the package directory)
        ('share/' + package_name + '/maps', glob('patrolling_robot_simulation/maps/*')),

        # Models directory (if you need models to be included)
        ('share/' + package_name + '/models', glob('patrolling_robot_simulation/models/**/*', recursive=True))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='marstonvisuals@gmail.com',
    description='ROS2 Package for patrolling robot simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "patrolling_robot_node=patrolling_robot_simulation.patrolling_robot_node:main"
        ],
    },
)
