from setuptools import find_packages, setup

package_name = 'maze_solver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bhavya Shah',
    maintainer_email='bshah43@asu.edu',
    description='Maze-solving nodes: perceives the maze from the overhead camera and plans a path through it',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_digitizer = maze_solver.maze_digitizer:main',
            'path_planner = maze_solver.path_planner:main',
        ],
    },
)
