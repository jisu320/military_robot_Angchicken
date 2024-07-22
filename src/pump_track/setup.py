import os
from glob import glob
from setuptools import setup
package_name = 'pump_track'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='jwchoi0017@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

                ###############spring##################
                'army_detect = pump_track.army_detection:main',
                'track_spring_check_top = pump_track.track_sping_top_check_node:main',
                ################summer#################
                'summer_track = pump_track.summer_track_node_3:main',
                'control = pump_track.control_node:main',
                'airdrop_pub = pump_track.airdrop_publish:main',
                ###############fall####################
                'marker_detect = pump_track.marker_detection:main',
                'track_fall_check = pump_track.track_check_node:main',
                ###############spring, fall##############
                'usb_cam = pump_track.usb_camera_node:main',
                'usb_cam_2 = pump_track.usb_2camera_node:main',
                ##############spring, fall, winter#######
                'imu = pump_track.imu_node:main',
                ##################spring################
                'diffbot_cont = pump_track.diffbot_control:main',
                'diffbot_position = pump_track.diffbot_position_control:main',
                ##################winter################
                'track_winter = pump_track.track_winter:main',
                ##################remote################
                'remote_control = pump_track.remote_control:main',
                'depth_cam = pump_track.depth_camera:main',


        ],
    },
)
