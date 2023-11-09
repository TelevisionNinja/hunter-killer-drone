#!/usr/bin/env python3


import subprocess
import time
import os


modelsPath = os.path.abspath("../models/")

commands = [
    "MicroXRCEAgent udp4 -p 8888", # Micro XRCE-DDS Agent
    f"cd ~/PX4-Autopilot/ && GZ_SIM_RESOURCE_PATH={modelsPath} PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE=\"283.08,-136.22,3.86,0.00,0,-0.7\" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4", # PX4 SITL simulation
    "source /opt/ros/humble/setup.bash && ros2 run ros_gz_image image_bridge /camera",
    "cd ~/QGroundControl/ && ./QGroundControl.AppImage"
]

for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

    # Pause between each command
    time.sleep(1)
