from setuptools import find_packages, setup

package_name = 'aruco_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clinton Enwerem',
    maintainer_email='me@clintonenwerem.com',
    description='ROS 2 package for ArUco marker detection and pose estimation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detect_aruco = aruco_pose_estimation.detect_aruco:main',
            'estimate_aruco_pose = aruco_pose_estimation.estimate_aruco_pose:main',
            'aruco_pose_visualizer = aruco_pose_estimation.aruco_pose_visualizer:main',
        ],
    },
)
