import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'yolobot'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/models',
            ['models/yolov8s.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Holzmann Kevin',
    maintainer_email='kevintoni@gmx.at',
    description='The yolobot package is bachelor thesis meant to implement yolo vision into the turtlebot4.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detect = yolobot.yolo_detect:main',
            'obj_loc = yolobot.obj_loc:main',
            'state_machine = yolobot.state_machine:main',
        ],
    },
)