source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
source /home/j11218cpu-i9/Desktop/sentry_RMUC_defend/install/setup.bash
sudo chmod 777 /dev/ttyUSBRM
sudo chmod 777 /dev/ttyUSB1
cmds=(  #"ros2 launch rm_vision_bringup vision_bringup.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 launch rm_bringup rm_bringup.launch.py"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
	"ros2 launch point_lio mapping_mid360.launch.py"
	"ros2 launch fast_lio mapping.launch.py"
	"ros2 launch sr_omni_pid_pursuit_controller-main launch.py"
	"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	"ros2 launch icp_registration icp.launch.py"
	"ros2 launch fake_vel_transform fake_vel_transform.launch.py"
	"ros2 launch rm_navigation bringup_launch.py"
	#"ros2 run fake_publisher fake_publisher_node"
	"ros2 run rm_serial_driver nav_to_pose_client_node"
	"ros2 launch rm_serial_driver serial_driver.launch.py"
	"ros2 run ROS2_bt ROS2_bt"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done

#"ros2 launch rm_bringup rm_bringup.launch.py"   "ros2 launch rm_serial_driver serial_driver.launch.py" source /opt/ros/humble/setup.bash
#"ros2 run ROS2_bt ROS2_bt" "ros2 launch rm_vision_bringup vision_bringup.launch.py"
#"ros2 run rm_serial_driver nav_to_pose_client_node" "ros2 launch point_lio point_lio.launch.py"
#"ros2 launch fast_lio  mapping.launch.py"
#"ros2 launch rm_bringup rm_bringup.launch.py"
#"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
#"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	#"ros2 launch icp_registration icp.launch.py"
	#"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	#"ros2 launch rm_navigation bringup_launch.py"
	#"ros2 launch rm_serial_driver serial_driver.launch.py"
	#"ros2 run rm_serial_driver nav_to_pose_client_node"
	#"ros2 run ROS2_bt ROS2_bt
#"ros2 launch Point-LIO point_lio.launch.py"
