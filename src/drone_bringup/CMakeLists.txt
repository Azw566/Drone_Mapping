cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

# Install directories into the ROS2 share folder.
# After `colcon build`, these are accessible via:
#   ros2 pkg prefix drone_bringup  →  install/drone_bringup/
#
# This matters because launch files use FindPackageShare("drone_bringup")
# to locate config files, models, and worlds at runtime.
install(DIRECTORY
  launch
  config
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python utilities/nodes and shell scripts
install(PROGRAMS
  scripts/lidar_enricher.py
  scripts/launch_px4_instance.sh
  scripts/save_arena_map.py
  scripts/gcs_heartbeat.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
