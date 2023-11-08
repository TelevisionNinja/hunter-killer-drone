cd ../workspace/src/
git clone https://github.com/PX4/px4_msgs.git
cd ..
source /opt/ros/humble/setup.bash
colcon build

# rename the original default.sdf file
mv ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf.original
# use the default.sdf file from this repo
cp ../worlds/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
