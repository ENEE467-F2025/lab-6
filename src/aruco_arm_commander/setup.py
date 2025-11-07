from setuptools import find_packages, setup

package_name = 'aruco_arm_commander'

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
    description='ROS 2 package for moving the tool frame of the UR to the pose of a detected ArUco tag.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_aruco = aruco_arm_commander.move_to_aruco:main'
        ],
    },
)
