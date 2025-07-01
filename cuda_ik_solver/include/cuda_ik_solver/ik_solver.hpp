// #pragma once

// #include <vector>
// #include <geometry_msgs/msg/pose.hpp>

// namespace cuda_ik_solver {

// std::vector<double> solveIKCUDA(const geometry_msgs::msg::Pose& target_pose);

// }
// ik_solver.hpp

#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace cuda_ik_solver {
  std::vector<double> solveIKCUDA(const geometry_msgs::msg::Pose& target_pose,
                                   const std::vector<double>& current_joint_angles);
}
