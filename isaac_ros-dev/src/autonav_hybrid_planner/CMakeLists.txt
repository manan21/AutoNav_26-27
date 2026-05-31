cmake_minimum_required(VERSION 3.8)
project(autonav_hybrid_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

set(dep_pkgs
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav2_util
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
    tf2)

add_library(${PROJECT_NAME}_core SHARED
  src/local_then_straight_planner.cpp)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_core ${dep_pkgs})

install(TARGETS ${PROJECT_NAME}_core
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

pluginlib_export_plugin_description_file(nav2_core autonav_hybrid_planner.xml)

ament_package()
