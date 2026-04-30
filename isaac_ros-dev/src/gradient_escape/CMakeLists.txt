cmake_minimum_required(VERSION 3.5)
project(gradient_escape)

set(lib_name ${PROJECT_NAME}_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dep_pkgs
    rclcpp
    rclcpp_action
    nav2_behaviors
    nav2_core
    nav2_msgs
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    pluginlib
    tf2
    tf2_ros
    tf2_geometry_msgs)

# === Build ===

add_library(${lib_name} SHARED
            src/gradient_escape.cpp)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_core gradient_escape.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
