
colcon build --symlink-install

	term 1
source install/setup.bash   # if you use the install tree
ros2 launch drone_bringup simulation.launch.py use_rviz:=true


	term 2 (drone 1)
export XRCE_DDS_CONFIG=udp://127.0.0.1:8888
cd ~/px4_workspace/PX4-Autopilot
PX4_GZ_MODEL=x500_d1 PX4_GZ_WORLD=maze PX4_GZ_MODEL_ALLOW_SPAWN=0 \
  make px4_sitl gz_x500_vision
  
	term 3 (drone 2)
export XRCE_DDS_CONFIG=udp://127.0.0.1:8889
cd ~/px4_workspace/PX4-Autopilot
PX4_GZ_MODEL=x500_d2 PX4_GZ_WORLD=maze PX4_GZ_MODEL_ALLOW_SPAWN=0 \
  make px4_sitl gz_x500_vision

	term 4 
./MicroXRCEAgent udp4 -p 8888   # d1
./MicroXRCEAgent udp4 -p 8889   # d2



