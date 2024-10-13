cmds=(  
    "roslaunch auto_nav map_server.launch"
    "roslaunch livox_ros_driver2 msg_MID360.launch"
    "rosrun livox_to_pointcloud2 livox_to_pointcloud2_node"
    "roslaunch pointcloud_to_laserscan pointcloud_to_laserscan.launch"
    "roslaunch fast_lio mapping_mid360.launch"
    "roslaunch auto_nav amcl.launch"
    "roslaunch auto_nav navi_simple_meca_car_pid.launch"
    "roslaunch TX Communication.launch"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source ./devel/setup.bash;$cmd;exec bash;"
	sleep 0.7
done
