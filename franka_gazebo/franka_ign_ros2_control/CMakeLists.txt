cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_ign_ros2_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Franka 0.13.3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Citadel")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")

else()
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")
endif()

find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

include_directories(include
${franka_ign_ros2_control_INCLUDE_DIRS} #추가!!!!

)

add_library(${PROJECT_NAME}-system SHARED
  src/ign_ros2_control_plugin.cpp
  src/model_kdl.cpp
)

target_link_libraries(${PROJECT_NAME}-system
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::register
)
ament_target_dependencies(${PROJECT_NAME}-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
  franka_hardware
  Franka
  urdf
  kdl_parser
  tf2_geometry_msgs
  tf2_eigen
)

#########

add_library(ign_hardware_plugins SHARED
  src/ign_system.cpp
  src/model_kdl.cpp
)
ament_target_dependencies(ign_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
  franka_hardware
  Franka
  urdf
  kdl_parser
  tf2_geometry_msgs
  tf2_eigen
)
target_link_libraries(ign_hardware_plugins
  ignition-gazebo${IGN_GAZEBO_VER}::core
)

## Install
install(TARGETS
  ign_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}-system ign_hardware_plugins)

# Install directories
install(TARGETS ${PROJECT_NAME}-system
  DESTINATION lib
)

pluginlib_export_plugin_description_file(franka_ign_ros2_control ign_hardware_plugins.xml)

# Setup the project
ament_package()
