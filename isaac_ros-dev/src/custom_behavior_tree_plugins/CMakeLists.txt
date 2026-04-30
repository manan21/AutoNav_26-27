cmake_minimum_required(VERSION 3.5)
project(custom_behavior_tree_plugins)

set(lib_name gradient_escape_core)

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
find_package(behaviortree_cpp_v3 REQUIRED)

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
    tf2_geometry_msgs
    behaviortree_cpp_v3)

# === Behavior server plugin: GradientEscape ===

add_library(${lib_name} SHARED
            src/gradient_escape.cpp)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${lib_name} ${dep_pkgs})

# === BT node plugin: GoalBender ===

add_library(autonav_goal_bender_bt_node SHARED
            src/goal_bender.cpp)

target_include_directories(autonav_goal_bender_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonav_goal_bender_bt_node ${dep_pkgs})

# === Installation ===

install(TARGETS ${lib_name} autonav_goal_bender_bt_node
        DESTINATION lib)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_core gradient_escape.xml)
ament_package()
