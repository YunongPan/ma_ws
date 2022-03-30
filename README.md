# ma_ws
Repository for MA RWTH


roslaunch urbant_simulation start.launch gpu:=false gazebo_gui:=true gps_switch:=true lidar_switch:=true world:=city


roslaunch hdl_localization hdl_localization_urbant.launch  (egal welche Karte geladet wird)


roscd hdl_localization/rviz


rviz -d hdl_localization.rviz


roslaunch geonav_transform geonav_transform.launch


rosrun hdl_map_update map_update_dev (development)


rosrun hdl_map_update map_update


rosservice call /set_gps_pos


rosrun teleop_twist_keyboard teleop_twist_keyboard.py
