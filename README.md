# ma_ws
Repository for MA RWTH


roslaunch urbant_simulation start.launch gpu:=false gazebo_gui:=true gps_switch:=true lidar_switch:=true world:=city


roslaunch hdl_localization hdl_localization.launch


roscd hdl_localization/rviz


rviz -d hdl_localization.rviz
