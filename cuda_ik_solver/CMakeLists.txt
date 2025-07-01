cmake_minimum_required(VERSION 3.16)  # Updated for better CUDA support
project(cuda_ik_solver)

# Set policy to silence CMP0104 warning (new in CMake 3.18)
cmake_policy(SET CMP0104 NEW)  # 

set(CMAKE_CUDA_ARCHITECTURES "89")  # RTX 4060 = sm_89
enable_language(CUDA)

# Compiler warnings (optional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(CUDA REQUIRED)

# Enable CUDA
enable_language(CUDA)

# CUDA-specific configuration for RTX 4060 (sm_89)
set(CUDA_NVCC_FLAGS
    -arch=sm_89                   # RTX 4060 architecture
    -O3                           # Max optimizations
    --default-stream per-thread   # Better ROS 2 compatibility
    --ptxas-options=-v            # Verbose assembly output
    -lineinfo                     # For profiling
    --expt-relaxed-constexpr      # Allow constexpr in device code
)

include_directories(
    include
    ${CUDA_INCLUDE_DIRS}          # Explicit CUDA includes
)

# Build CUDA library
cuda_add_library(ik_cuda SHARED
    src/ik_solver.cu
)
ament_target_dependencies(ik_cuda
    geometry_msgs
)
# Build ROS 2 node
add_executable(ik_node src/ik_node.cpp)
target_link_libraries(ik_node
    ik_cuda
    ${CUDA_LIBRARIES}             # Explicit CUDA linking
)

# ROS 2 dependencies
ament_target_dependencies(ik_node
    rclcpp
    geometry_msgs
    trajectory_msgs
)

# Install targets
install(TARGETS
    ik_node
    ik_cuda
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch/scripts
install(
    DIRECTORY
        launch
        scripts
    DESTINATION
        share/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()