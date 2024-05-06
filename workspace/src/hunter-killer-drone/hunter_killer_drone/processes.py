#!/usr/bin/env python3


import subprocess
import time
import os


modelsPath = os.path.abspath("../models/")

# model = "fixed_wing"
model = "x500_depth"

# x
# + back
# - front

# y
# + left front
# - right back
pose = "283.08, -140, 3.86, 0, 0, -0.69"

# PX4_SIM_MODEL v1.15
# PX4_GZ_MODEL v1.14
commands = [
    "MicroXRCEAgent udp4 -p 8888", # Micro XRCE-DDS Agent
    f"cd ~/PX4-Autopilot/ && GZ_SIM_RESOURCE_PATH={modelsPath} PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE=\"{pose}\" PX4_GZ_MODEL={model} ./build/px4_sitl_default/bin/px4", # PX4 SITL simulation
    "source /opt/ros/humble/setup.bash && ros2 run ros_gz_image image_bridge /camera /depth_camera", # gazebo ros bridge
    "cd ~/QGroundControl/ && ./QGroundControl.AppImage" # QGroundControl
]

for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

    # Pause between each command
    time.sleep(1)
