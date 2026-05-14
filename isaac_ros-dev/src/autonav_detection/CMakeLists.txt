cmake_minimum_required(VERSION 3.18)  # 3.18+ recommended for CUDA language
project(autonav_detection LANGUAGES CXX CUDA)

# Warnings for host code only. -Wpedantic chokes on the auto-generated
# stub.c / cudafe1.cpp files NVCC produces for CUDA targets ("style of
# line directive is a GCC extension"), so scope these to CXX language
# only — NVCC compilation passes won't see them.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

# ---- ROS & deps ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

# Force -O3 + -DNDEBUG for the host-side host code regardless of the
# workspace's CMAKE_BUILD_TYPE. Same reasoning as grade_detector below
# (see its block): colcon's default build type is empty → GCC defaults
# to -O0, which strangles the per-frame CUDA/OpenCV wrapper here
# (cv_bridge copies, integral image setup, post-processing). The .cu
# compilation pass uses CMake's CUDA flags so we scope this to CXX only.
target_compile_options(line_detector PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-O3>
  $<$<COMPILE_LANGUAGE:CXX>:-DNDEBUG>
)

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
  nav_msgs
  std_msgs
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

# Eigen 3.4 on aarch64 (Jetson) emits -Wclass-memaccess warnings from
# its own NEON intrinsics (PacketMath.h), not our code. Known false
# positive — Eigen's eigen_packet_wrapper isn't actually non-trivial in
# any meaningful sense, but GCC flags the memcpy. Suppress for this
# target only.
#
# Force -O3 + -DNDEBUG regardless of the workspace's CMAKE_BUILD_TYPE.
# The per-cell PCA in pca_pipeline.cpp is the hottest loop in the
# perception stack and depends entirely on Eigen's templated math
# being inlined — without -O3 it runs ~50-100× slower (measured 228 ms
# per scan unoptimized vs ~5 ms expected). colcon's default build type
# is empty, which means GCC defaults to -O0; we never want that for
# this target.
target_compile_options(grade_detector PRIVATE
  -Wno-class-memaccess
  -O3
  -DNDEBUG
)

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
