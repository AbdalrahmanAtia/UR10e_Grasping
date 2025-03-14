# app/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'app'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Add other data files if necessary
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'scipy', 'open3d', 'torch'],
    zip_safe=True,
    maintainer='artc',
    maintainer_email='abdelrohman.atia@gmail.com',
    description='Coordinator and related nodes package',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator = app.coordinator:main',
            'grasp_detector_controller = app.grasp_detector_controller:main',
            'robot_controller_moveit = app.robot_controller_moveit:main',
            'robot_controller_rtde = app.robot_controller_rtde:main',
            'gripper_controller = app.gripper_controller:main',
            'transformation = app.transformation:main',

            # Add other executables as needed
        ],
    },
)


