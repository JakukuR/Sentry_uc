#!/bin/bash
cmds=( 
	"ros2 run nav2_map_server map_saver_cli -f /home"
    "ros2 service call /map_save std_srvs/srv/Trigger"
)
for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2 
done
