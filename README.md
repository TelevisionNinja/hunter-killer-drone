# hunter-killer-drone
Firmware and simulation of a hunter killer drone

Dependencies
- ROS 2 Humble
- PX4
- Micro XRCE-DDS Agent
- OpenCV
- Ultralytics YOLOv8
- ros_gz

## Installation

Clone the repo
```bash
git clone https://github.com/TelevisionNinja/hunter-killer-drone.git
```

Go to the scripts folder and make everything executable
```bash
cd ./hunter-killer-drone/scripts
chmod +x *
```

Setup Ubuntu
```bash
./ubuntuSetup.sh
```

Build the repo
```bash
./build.sh
```

Launch the simulation
```bash
./launch.sh
```

## Controls

W: Up<br/>
S: Down<br/>
A: Yaw Left<br/>
D: Yaw Right<br/>

Up Arrow: Pitch Forward<br/>
Down Arrow: Pitch Backward<br/>
Left Arrow: Roll Left<br/>
Right Arrow: Roll Right<br/>

SPACE: Arm/disarm the drone
