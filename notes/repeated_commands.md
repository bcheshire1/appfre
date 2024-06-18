# Robot commands
1. ssh in to the RasPi: `ssh appfre_robot@192.168.0.100` (192.168.0.100 is IP address for appfre_network)
2. `cd docker`
3. `sudo docker run --rm -it --privileged -v /lib/modules:/lib/modules --network host --name appfreRobot appfre-robot:v1.8 bash`
4. `source install/setup.bash`
5. `sudo modprobe gs_usb`
6. `bash ./src/ugv_sdk/scripts/setup_can2usb.bash` (needs WiFi connection)
7. `bash ./src/ugv_sdk/scripts/bringup_can2usb_500k.bash` (might say "`Device or resource busy`", don't worry about it)
8. `ros2 launch bunker_base bunker_base.launch.py`

# LIDAR commands
1. ssh in to the RasPi: `ssh appfre_robot@192.168.0.100` (192.168.0.100 is IP address for appfre_network)
2. `cd docker`
3. `sudo docker run --rm -it --privileged -v /lib/modules:/lib/modules --network host --name appfreLidar appfre-robot:v1.8 bash`
4. `source install/setup.bash`
5. `source ./src/rplidar_ros/scripts/create_udev_rules.sh` (From ros2_ws directory, don't worry about any unrecognized services, or commands not found)
6. `ros2 launch rplidar_ros rplidar_a2m12_launch.py frame_id:=laser_frame`

# PiGI commands
1. ssh into the RasPi: `ssh appfre@192.168.0.103` (192.168.0.103 is IP address for appfre_network)
2. Run Geiger.py script on the robot RasPi: `python3 Geiger.py`
3. Run Geiger_receive.py on the Base PC: `python3 Geiger_receive.py` (fromt the src directory)

## Map Saver
```
ros2 run nav2_map_server map_saver_cli
```
