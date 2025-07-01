#ifndef CUDA_IK_SOLVER_HPP
#define CUDA_IK_SOLVER_HPP

#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace cuda_ik_solver {

  std::vector<double> solveIKCUDA(const geometry_msgs::msg::Pose& target_pose,
                                   const std::vector<double>& current_joint_angles);
  void forwardKinematics(const std::vector<double>& joint_angles, float* ee_out);

}

#endif

// #pragma once
// #include <geometry_msgs/msg/pose.hpp>
// #include <vector>

// namespace cuda_ik_solver {
//   std::vector<double> solveIKCUDA(const geometry_msgs::msg::Pose& target_pose,
//                                    const std::vector<double>& current_joint_angles);
// }
