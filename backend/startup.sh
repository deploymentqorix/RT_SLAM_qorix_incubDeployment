#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Execute the main Python script
exec python3 data_streamer/main.py
