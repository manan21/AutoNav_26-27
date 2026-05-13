cmake_minimum_required(VERSION 3.5)
project(local_mirror_layer)

set(lib_name ${PROJECT_NAME}_core)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    nav2_common
    pluginlib
    nav_msgs
    tf2_ros)

add_library(${lib_name} SHARED
            src/local_mirror_layer.cpp)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS ${lib_name}
        DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_costmap_2d local_mirror_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
