from setuptools import find_packages, setup

package_name = 'final_project'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch_final.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/maze.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/maze1_smallest.wbt']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
]))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='oliviaadjm@gmail.com',
    description='Simulation for CS 460/560 Project 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],    
        'console_scripts': ['final_project = final_project.final_project:main']
    },
)
