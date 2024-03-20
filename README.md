## Autonomous Path Planning For Radioactive Environments (appfre) using the Bunker Mini robot by AgileX

Ensure you have ros2 installed as described in https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

You will also need to instal the SLAM toolbox, Nav2, and others

Clone this repo into the src directory of your colcon workspace
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
```
git clone https://github.com/bcheshire1/bunker_mini.git
```
Build the package using colcon from the root of your workspace
```
cd ..
```
```
colcon build --symlink-install
```
Source the workspace
```
source install/setup.bash
```
From the root of your workspace, run an example launch file
```
ros2 launch bunker_mini launch_nav2.launch.py world:=./src/bunker_mini/worlds/room.world
```
