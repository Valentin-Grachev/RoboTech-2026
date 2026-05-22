from glob import glob
from setuptools import find_packages, setup


package_name = 'tb3_astar_fuzzy'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Fuzzy controller and A* path planning over a custom SLAM occupancy grid.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_mapper = tb3_astar_fuzzy.slam_mapper_node:main',
            'fuzzy_controller = tb3_astar_fuzzy.fuzzy_controller_node:main',
            'astar_planner = tb3_astar_fuzzy.astar_planner_node:main',
        ],
    },
)
