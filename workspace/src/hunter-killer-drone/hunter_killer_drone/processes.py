#!/usr/bin/env python3


import subprocess
import time


commands = [
    "MicroXRCEAgent udp4 -p 8888", # Micro XRCE-DDS Agent
    "cd ~/PX4-Autopilot/ && make px4_sitl gz_x500" # PX4 SITL simulation
]

for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

    # Pause between each command
    time.sleep(1)
