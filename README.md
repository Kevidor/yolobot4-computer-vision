# TurtleBot4 Machine Vision with YOLOvX

## Overview

This repository contains the implementation and experiments for a Bachelor Thesis focused on machine vision using YOLO-based object detection on a TurtleBot4 platform. The goal is to enable real-time perception and environment understanding for mobile robotics applications.

The project integrates computer vision, robotics, and AI to detect, classify, and potentially track objects in a dynamic environment.

---

## Features

* Real-time object detection using YOLO
* Integration with TurtleBot4
* ROS/ROS2-based communication
* Designed for experimentation and research

---

## Project Structure

```
.
├── build
│   ├── COLCON_IGNORE
│   └── yolobot
│       ├── build
│       │   └── lib
│       ├── colcon_build.rc
│       ├── colcon_command_prefix_setup_py.sh
│       ├── colcon_command_prefix_setup_py.sh.env
│       ├── install.log
│       ├── prefix_override
│       │   ├── __pycache__
│       │   └── sitecustomize.py
│       └── yolobot.egg-info
│           ├── dependency_links.txt
│           ├── entry_points.txt
│           ├── PKG-INFO
│           ├── requires.txt
│           ├── SOURCES.txt
│           ├── top_level.txt
│           └── zip-safe
├── install
│   ├── COLCON_IGNORE
│   ├── local_setup.bash
│   ├── local_setup.ps1
│   ├── local_setup.sh
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── local_setup.zsh
│   ├── setup.bash
│   ├── setup.ps1
│   ├── setup.sh
│   ├── setup.zsh
│   └── yolobot
│       ├── lib
│       │   ├── python3.10
│       │   └── yolobot
│       └── share
│           ├── ament_index
│           ├── colcon-core
│           └── yolobot
├── LICENSE
├── log
│   ├── build_2026-07-05_19-42-20
│   │   ├── events.log
│   │   ├── logger_all.log
│   │   └── yolobot
│   │       ├── command.log
│   │       ├── stderr.log
│   │       ├── stdout.log
│   │       ├── stdout_stderr.log
│   │       └── streams.log
│   ├── COLCON_IGNORE
│   ├── latest -> latest_build
│   └── latest_build -> build_2026-07-05_19-42-20
├── README.md
└── src
    └── yolobot
        ├── models
        │   └── yolov8s.pt
        ├── package.xml
        ├── resource
        │   └── yolobot
        ├── setup.cfg
        ├── setup.py
        ├── test
        │   ├── test_copyright.py
        │   ├── test_flake8.py
        │   └── test_pep257.py
        └── yolobot
            ├── __init__.py
            ├── obj_loc.py
            ├── person_follow.py
            ├── state_machine.py
            ├── target_gen.py
            ├── yolo_basic.py
            ├── yolo_detect.py
            └── yolo_viz.py
```

---
## Prerequisites

This package requires ROS 2 Humble or later. Follow the official installation guide:

- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

After installing ROS 2, source the setup file:

```bash
source /opt/ros/humble/setup.bash
```

All other dependencies will be installed during the compilation process.


## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/Kevidor/yolobot4-computer-vision.git
   ```
2. Install dependencies:

   ```
   <add your setup steps here>
   ```
3. Build the workspace (if using ROS):

   ```bash
   colcon build --packages-select yolobot
   ```
   or build everything:
   ```bash
   colcon build
   ``

---

## Usage

Run the main application:

```
<command to launch your system>
```

Example:

```
<ros2 launch ...>
```

---

## Tutorial

A step-by-step tutorial will be provided to:

* Set up the TurtleBot4
* Configure the camera and sensors
* Run YOLO-based detection
* Interpret results

(Replace this section with detailed instructions as the project evolves.)

---

## Results

* Detection accuracy: <to be added>
* Performance (FPS): <to be added>
* Example outputs: <images / videos>

---

## Contributing

Contributions are welcome. Please:

* Fork the repository
* Create a feature branch
* Submit a pull request

---

## Maintainer

* Kevin Holzmann
  (email: <kevintoni@gmx.at>)

---

## License

This project is licensed under the Apache License 2.0.
See the LICENSE file for details.

---

## Acknowledgements

* TurtleBot4 platform
* YOLO framework
* Open-source robotics and computer vision community

---

## Notes

This repository is part of a Bachelor Thesis and is intended for research and educational purposes. The structure and content may change frequently.