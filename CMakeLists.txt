cmake_minimum_required(VERSION 3.5)

project(system_monitor_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PythonInterp REQUIRED)

#################
# Setup targets #
#################

include_directories(include ${rclcpp_INCLUDE_DIRS})

# Declare system_monitor library
add_library(system_monitor
  src/system_monitor.cpp
)

# Add node executable
add_executable(system_monitor_node
  src/system_monitor_node.cpp
)
target_link_libraries(system_monitor_node system_monitor)

#################################
# ament specific configuration ##
#################################

# Add node dependencies
ament_target_dependencies(system_monitor_node px4_msgs rclcpp std_msgs)

# Export information to downstream packages
ament_export_dependencies(rclcpp rosidl_default_runtime system_monitor)
ament_export_include_directories(include ${px4_msgs_INCLUDE_DIRS})
ament_export_libraries()

############
# Install ##
############

# Install artifacts
install(DIRECTORY include DESTINATION include)
install(TARGETS system_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS system_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

############
# Testing ##
############

# TODO

ament_package()
