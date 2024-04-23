## Autonomous Path Planning For Radioactive Environments (appfre) using the Bunker Mini robot by AgileX

**Ensure you have ros2 installed as described in https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html**

---

You will also need to install: 
- SLAM toolbox `sudo apt install ros-humble-slam-toolbox`
- Nav2 `sudo apt install \ ros-humble-navigation2 \ ros-humble-nav2-bringup`
- colcon `sudo apt install python3-colcon-common-extensions`
- gazebo `curl -sSL http://get.gazebosim.org | sh`
- and others `Add relevant installs here for future use`

Clone this repo into the src directory of your colcon workspace
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
```
git clone https://github.com/bcheshire1/appfre.git
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
Then in a new terminal, from the root of your directory again, run the explore.py file to make the robot explore the environment and build up a map using SLAM
```
python3 ./src/bunker_mini/src/explore.py
```
