from setuptools import setup
import os
from glob import glob

package_name = 'automated_docking_project'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam Shoni',
    maintainer_email='samshoni@example.com',
    description='Automated Docking Simulation for Mobile Robots in ROS2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = automated_docking_project.battery_monitor:main',
            'docking_controller = automated_docking_project.docking_controller:main',
        ],
    },
)

