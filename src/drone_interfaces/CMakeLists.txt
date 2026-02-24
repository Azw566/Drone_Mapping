cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

# Default to C++17 (ROS2 Humble standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# These are the standard msg packages we depend on.
# Every type you reference in your .msg files (e.g., geometry_msgs/PoseStamped)
# must have its package found here.
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# THIS is the magic function. It takes your .msg and .srv files and
# generates Python classes + C++ structs at build time.
#
# After building, you can do:
#   from drone_interfaces.msg import ArucoDetection
#   from drone_interfaces.srv import RegisterPOI
#
# The DEPENDENCIES list must match every external msg package used in your definitions.
rosidl_generate_interfaces(${PROJECT_NAME}
  # Message definitions
  "msg/ArucoDetection.msg"
  "msg/FrontierList.msg"
  "msg/DroneState.msg"
  # Service definitions
  "srv/AssignFrontier.srv"
  "srv/RegisterPOI.srv"
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

ament_package()
