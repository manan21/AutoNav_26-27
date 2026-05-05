cmake_minimum_required(VERSION 3.18)  # 3.18+ recommended for CUDA language
project(autonav_detection LANGUAGES CXX CUDA)

# Warnings for host code
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- ROS & deps ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonav_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_geometry REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- CUDA Toolchain (modern; line_detector only) ----
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# Jetson Orin Nano is Ampere (SM 87)
set(CMAKE_CUDA_ARCHITECTURES 87)

# ============================================================
# Target 1: line_detector  (existing, CUDA-accelerated)
# ============================================================
add_executable(line_detector
  src/line/node.cpp
  src/line/detection.cpp
  src/line/cuda.cu
)

target_compile_features(line_detector PUBLIC cxx_std_17)
set_target_properties(line_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(line_detector
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${image_geometry_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(line_detector
  rclcpp
  autonav_interfaces
  cv_bridge
  tf2_ros
  tf2_eigen
  Eigen3
  image_geometry
  std_srvs
  tf2_geometry_msgs
)

# ---- Link CUDA & NPP (modern imported targets) ----
set(_cuda_npp_targets CUDA::cudart CUDA::nppist CUDA::nppc)

# Fallback if CUDAToolkit imported targets don't define nppist on this CMake/Toolkit combo.
foreach(tgt IN LISTS _cuda_npp_targets)
  if(NOT TARGET ${tgt})
    list(APPEND _cuda_fallback_libs
      /usr/local/cuda/targets/aarch64-linux/lib/libcudart.so
      /usr/local/cuda/targets/aarch64-linux/lib/libnppist.so
      /usr/local/cuda/targets/aarch64-linux/lib/libnppc.so
    )
    set(_use_fallback_libs TRUE)
    break()
  endif()
endforeach()

if(_use_fallback_libs)
  target_link_libraries(line_detector ${_cuda_fallback_libs})
  target_link_options(line_detector PRIVATE -Wl,-rpath,/usr/local/cuda/targets/aarch64-linux/lib)
else()
  target_link_libraries(line_detector ${_cuda_npp_targets})
  target_link_options(line_detector PRIVATE -Wl,-rpath,$<TARGET_FILE_DIR:CUDA::cudart>)
endif()

# ============================================================
# Target 2: grade_detector  (NEW; pure C++ + Eigen, no CUDA)
# Pipeline: subscribe to /cloud_all_fields_fullframe (PointCloud2),
# run PCA grade detection (Steps 1-5 of terrain-grade-layer-plan.md),
# publish /scan_pca_filtered_points (PointCloud2, xyz only) for the
# Nav2 ObstacleLayer to consume.
# ============================================================
add_executable(grade_detector
  src/grade/pca_node.cpp
  src/grade/pca_pipeline.cpp
)

target_compile_features(grade_detector PUBLIC cxx_std_17)
set_target_properties(grade_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(grade_detector
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(grade_detector
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  Eigen3
)

# ---- Install ----
install(TARGETS line_detector grade_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ---- Tests / Linters ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
