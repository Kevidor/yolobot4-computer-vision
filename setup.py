import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'image_detection'

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
    ("share/" + PACKAGE_NAME, ["package.xml"]),
    ("share/" + PACKAGE_NAME + "/launch", glob.glob("launch/*.launch.py")),
    ("share/" + PACKAGE_NAME + "/models", glob.glob("models/*")),
]

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Holzmann Kevin',
    maintainer_email='kevintoni@gmx.at',
    description='This package implements a ROS2 node for image detection using a pre-trained YOLOv26 model.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_detection_node = image_detection.image_detection_node:main'
        ],
    },
)