# Save the README content to a .txt file
readme_txt_content = """\
Havoc PTZ
=========

Havoc PTZ is a Python-based system for controlling a PTZ (Pan-Tilt-Zoom) camera using the ONVIF standard.
It provides basic camera control functionality and is designed for integration into larger robotics or surveillance systems.

Features
--------
- ONVIF-based PTZ control
- ROS2 integration support
- Example scripts for moving the camera
- Dockerfile for containerized deployment

Getting Started
---------------

**Prerequisites**

Ensure you have the following installed:

- Python 3.8
- onvif_zeep
- numpy
- opencv-python
- rclpy (if using ROS2 integration)
- setuptools

You can install dependencies using:

    pip install onvif_zeep numpy opencv-python

If you're using ROS2 (optional but recommended), also install ROS2 and source your environment:

    source /opt/ros/<your_distro>/setup.bash

**Installation**

Clone or unzip this repository, then navigate to the directory:

    cd havoc_ptz-main

You can optionally build and install the ROS2 package if you're using it:

    colcon build
    source install/setup.bash

Usage
-----

To test basic PTZ movement and confirm installation, run:

    python3 scripts/move_test.py

Make sure your PTZ camera is connected and accessible over the network, and edit the IP or credentials in the script if needed.

Project Structure
-----------------

scripts/
├── move_test.py         # Primary test script for PTZ movement
├── ptz_controller.py    # ONVIF control logic
├── ptz_node.py          # ROS2-compatible node
launch/
├── ptz_launch.py        # ROS2 launch file
utils/
├── republish_rtmp_stream.sh # Utility for RTMP stream republishing

License
-------

This project currently has no license. All rights reserved by the author unless otherwise specified.

Author
------

Zachary Serlin 
Kevin Becker
"""

