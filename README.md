# CUDA Accelerated IK Solver for 6 DOF Manipulator

This package implements an efficient GPU-based Inverse Kinematics (IK) solver using CUDA to compute joint angles for a 6-DOF robotic arm in ROS 2. It supports both position and orientation targets and publishes optimal trajectories to a controller.

### Package Structure
cuda_ik_solver/
    include/
        ik_solver.hpp
    src/
        ik_node.cpp     # ROS2 Node
        ik_solver.cu    # CUDA IK Solver
    CMakeLists.txt
    package.xml
    README.md

## 🔧 Dependencies

- ROS 2 Humble or newer
- CUDA Toolkit ≥ 11.0
- CMake ≥ 3.10
- Compatible robot (e.g., UR5, UR10)
- Robot simulation (e.g., Gazebo or RViz)

---

## Build Instructions

```bash
# Clone this package into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/Vinothhk/CUDA-Accelerated-IK-Solver-for-a-6-DOF-Manipulator.git cuda_ik_solver

# Build with CUDA support
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_CUDA_STANDARD=14
source install/setup.bash
```

## 
Launch ur sim:
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py 
```

Run IK solver:
```
ros2 run cuda_ik_solver ik_node 
```
Give Pose:
```
ros2 topic pub /ee_goal geometry_msgs/PoseStamped "header:
  frame_id: 'base_link'
pose:
  position: {x: 0.4, y: 0.1, z: 0.3}
  orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}"
```