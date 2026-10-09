from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['robot', 'robot.master', 'robot.camera', 'robot.lidar', 'robot.signs', 'robot.motion', 'robot.lane', 'robot.obstacle', 'robot.turn', 'robot.uturn', 'robot.parking', 'robot.parallel_parking', 'robot.common'],
    package_dir={'': '..'},
))
