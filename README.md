## Autonomous Path Planning For Radioactive Environments (appfre) using the Bunker Mini robot by AgileX

**Ensure you have ros2 installed as described in https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html**

---

You will also need to install: 
- Nav2 `sudo apt install \ ros-humble-navigation2 \ ros-humble-nav2-bringup`
- colcon `sudo apt install python3-colcon-common-extensions`
- gazebo: Follow instructions at https://gazebosim.org/docs/fortress/install_ubuntu
- gazebo_ros `sudo apt install ros-humble-gazebo-ros-pkgs`
- xacro `sudo apt install ros-humble-xacro`  

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
ros2 launch appfre launch_nav2.launch.py world:=./src/appfre/worlds/room.world
```
Then in a new terminal, from the root of your directory again, run the explore.py file to make the robot explore the environment and build up a map using SLAM
```
python3 ./src/bunker_mini/src/explore.py
```
