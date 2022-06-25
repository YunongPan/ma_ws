# ma_ws
Repository for MA RWTH. Compared with the original Repository, the main differences are:
1. Add hdl_map_update package.
2. Modifiy the globalmap_server_nodelet.cpp in the hdl_localization/apps
## Installation
1. `cd ~`
2. `git clone https://github.com/YunongPan/ma_ws.git`
3. `cd ~/ma_ws`
4. `catkin_make`
## Rasterization
1. Put .pcd file of big point cloud map into `/ma_ws/src/hdl_map_update/pcd_files`
2. Modify `original_file_pcd` in `/ma_ws/src/hdl_map_update/pcd_files/pcd_splitter.py` to match the name of
large point cloud map.
3. Modify `chunk_size`.
4. `cd ~/ma_ws/src/hdl_map_update/pcd_files`
5. `python pcd_splitter.py`

## Start map_update testing
1. Start testing with installing urbant_ws. Please see https://git.rwth-aachen.de/mobile-robotics/urbant/urbant_ws
2. Copy the `ma_515.world` file in `/ma_ws/src/hdl_map_update/world` into `/urbant_ws/src/urbant_simulation/worlds`
3. Modify the value of "world_name" in `urbant_ws/src/urbant_simulation/launch/world/city.launch` to "$(find urbant_simulation)/worlds/ma_515.world"
4. Open a terminal, `cd ~/urbant_ws`
5. `roslaunch urbant_simulation start.launch gpu:=false gazebo_gui:=true gps_switch:=true lidar_switch:=true world:=city`
6. Modify parameters in `/ma_ws/src/hdl_localization/apps/globalmap_server_nodelet.cpp` and `/ma_ws/src/hdl_map_update/src/map_update.cpp` 
7. Open a terminal, `cd ~/ma_ws`
8. `catkin_make`
9. Open a terminal, `cd ~/ma_ws`
10. `roslaunch hdl_localization hdl_localization_urbant.launch`
11. Open a terminal,`cd ~/ma_ws`
12. `roscd hdl_localization/rviz`
13. `rviz -d hdl_localization.rviz`
14. Open a terminal,`cd ~/ma_ws`
15. `roslaunch geonav_transform geonav_transform.launch`
16. Open a terminal,`cd ~/ma_ws`
17. `rosrun hdl_map_update map_update`
18. Open a terminal,`rosservice call /set_gps_pos`
18. Open a terminal,`rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control robot.

