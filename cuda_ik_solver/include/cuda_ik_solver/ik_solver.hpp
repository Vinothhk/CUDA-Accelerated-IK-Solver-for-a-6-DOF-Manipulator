#pragma once

#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace cuda_ik_solver {

std::vector<double> solveIKCUDA(const geometry_msgs::msg::Pose& target_pose);

}