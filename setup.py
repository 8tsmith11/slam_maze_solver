from setuptools import find_packages, setup
import glob

package_name = 'slam_maze_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trevor Smith',
    maintainer_email='8tsmith11@gmail.com',
    description='SLAM maze solver for Turtlebot 4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potential_field_nav_node = slam_maze_solver.potential_field_nav_node:main',
            'premapped_nav_node = slam_maze_solver.premapped_nav_node:main',
        ],
    },
)