cmake_minimum_required(VERSION 3.5)
project(custom_behavior_tree_plugins)

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
find_package(nav_msgs REQUIRED)
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
    nav_msgs
    pluginlib
    tf2
    tf2_ros
    tf2_geometry_msgs
    behaviortree_cpp_v3)

# === Behavior server plugin: GradientEscape ===

set(grad_lib gradient_escape_core)
add_library(${grad_lib} SHARED
            src/gradient_escape.cpp)

target_include_directories(${grad_lib}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${grad_lib} ${dep_pkgs})

# === Behavior server plugin: BreadcrumbReverse ===

set(breadcrumb_lib breadcrumb_reverse_core)
add_library(${breadcrumb_lib} SHARED
            src/breadcrumb_reverse.cpp)

target_include_directories(${breadcrumb_lib}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${breadcrumb_lib} ${dep_pkgs})

# === Behavior server plugin: InflationBackout ===

set(inflation_backout_lib inflation_backout_core)
add_library(${inflation_backout_lib} SHARED
            src/inflation_backout.cpp)

target_include_directories(${inflation_backout_lib}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${inflation_backout_lib} ${dep_pkgs})

# === BT node plugin: GoalBender ===

add_library(autonav_goal_bender_bt_node SHARED
            src/goal_bender.cpp)

target_compile_definitions(autonav_goal_bender_bt_node PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(autonav_goal_bender_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonav_goal_bender_bt_node ${dep_pkgs})

# === BT node plugin: PathSignificantlyChanged (PHASEB) ===

add_library(autonav_path_significantly_changed_bt_node SHARED
            src/path_significantly_changed_decorator.cpp)

target_compile_definitions(autonav_path_significantly_changed_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(autonav_path_significantly_changed_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonav_path_significantly_changed_bt_node ${dep_pkgs})

# === BT node plugin: IsForwardBlocked ===

add_library(autonav_is_forward_blocked_bt_node SHARED
            src/is_forward_blocked.cpp)

target_compile_definitions(autonav_is_forward_blocked_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(autonav_is_forward_blocked_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonav_is_forward_blocked_bt_node ${dep_pkgs})

# === BT node plugin: PathFootprintSafe ===

add_library(autonav_path_footprint_safe_bt_node SHARED
            src/path_footprint_safe_condition.cpp)

target_compile_definitions(autonav_path_footprint_safe_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(autonav_path_footprint_safe_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(autonav_path_footprint_safe_bt_node ${dep_pkgs})

# === Standalone node: breadcrumb_buffer ===

add_executable(breadcrumb_buffer src/breadcrumb_buffer.cpp)

target_include_directories(breadcrumb_buffer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(breadcrumb_buffer ${dep_pkgs})

# === Installation ===

install(TARGETS ${grad_lib}
                ${breadcrumb_lib}
                ${inflation_backout_lib}
                autonav_goal_bender_bt_node
                autonav_path_significantly_changed_bt_node
                autonav_is_forward_blocked_bt_node
                autonav_path_footprint_safe_bt_node
        DESTINATION lib)

install(TARGETS breadcrumb_buffer
        DESTINATION lib/${PROJECT_NAME})

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_core gradient_escape.xml)
pluginlib_export_plugin_description_file(nav2_core breadcrumb_reverse.xml)
pluginlib_export_plugin_description_file(nav2_core inflation_backout.xml)
ament_package()
