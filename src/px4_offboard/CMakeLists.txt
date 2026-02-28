cmake_minimum_required(VERSION 3.8)
project(px4_offboard)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (makes 'import px4_offboard' work at runtime)
ament_python_install_package(${PROJECT_NAME})

# Install thin wrapper scripts → lib/px4_offboard/ (where ros2 run looks)
install(PROGRAMS
  scripts/offboard_controller
  scripts/visual_odom_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
