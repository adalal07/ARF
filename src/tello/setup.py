from setuptools import find_packages, setup

package_name = 'tello'

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
    maintainer='ARtheboss',
    maintainer_email='30683624+ARtheboss@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tello_bringup = tello.node:main',
            'aruco_tf_broadcaster = tello.aruco_tf_broadcaster:main',
            'aruco_viser_visualizer = tello.aruco_viser_visualizer:main',
            'pid_controller = tello.aruco_controller:main',
            'gesture_controller = tello.gesture_controller:main',
            'hand_gesture_detector = tello.hand_gestures:main',
        ],
    },
)
