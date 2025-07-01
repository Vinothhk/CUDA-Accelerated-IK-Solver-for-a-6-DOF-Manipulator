#include <cmath>
#include <vector>
#include <iostream>
#include <geometry_msgs/msg/pose.hpp>
#include "cuda_ik_solver/ik_solver.hpp"

__device__ float compute_cost(const float* joint_angles, const float* target_pose) {
    // Dummy cost function for now — replace with FK comparison
    float cost = 0.0f;
    for (int i = 0; i < 6; ++i)
        cost += (joint_angles[i] - target_pose[i]) * (joint_angles[i] - target_pose[i]);
    return cost;
}

__global__ void evaluate_ik_solutions(float* candidate_angles, float* costs, float* target_pose, int num_candidates) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < num_candidates) {
        float* joint_angles = &candidate_angles[idx * 6];
        costs[idx] = computcompute_coste_cost(joint_angles, target_pose);
    }
}

std::vector<double> cuda_ik_solver::solveIKCUDA(const geometry_msgs::msg::Pose& target_pose) {
    constexpr int num_candidates = 256;
    float h_candidates[num_candidates * 6];
    float h_costs[num_candidates];
    // float h_target_pose[6] = {target_pose.position.x, target_pose.position.y, target_pose.position.z, 0, 0, 0};
    float h_target_pose[6] = {
        static_cast<float>(target_pose.position.x),
        static_cast<float>(target_pose.position.y),
        static_cast<float>(target_pose.position.z),
        0.0f, 0.0f, 0.0f
    };

    for (int i = 0; i < num_candidates * 6; ++i)
        h_candidates[i] = static_cast<float>(rand()) / RAND_MAX * 3.14f; // random angles

    float* d_candidates;
    float* d_costs;
    float* d_target_pose;
    cudaMalloc(&d_candidates, sizeof(float) * num_candidates * 6);
    cudaMalloc(&d_costs, sizeof(float) * num_candidates);
    cudaMalloc(&d_target_pose, sizeof(float) * 6);

    cudaMemcpy(d_candidates, h_candidates, sizeof(float) * num_candidates * 6, cudaMemcpyHostToDevice);
    cudaMemcpy(d_target_pose, h_target_pose, sizeof(float) * 6, cudaMemcpyHostToDevice);

    evaluate_ik_solutions<<<(num_candidates + 255)/256, 256>>>(d_candidates, d_costs, d_target_pose, num_candidates);

    cudaMemcpy(h_costs, d_costs, sizeof(float) * num_candidates, cudaMemcpyDeviceToHost);

    int best_idx = 0;
    for (int i = 1; i < num_candidates; ++i)
        if (h_costs[i] < h_costs[best_idx]) best_idx = i;

    std::vector<double> best_joint_angles(6);
    for (int j = 0; j < 6; ++j)
        best_joint_angles[j] = h_candidates[best_idx * 6 + j];

    cudaFree(d_candidates);
    cudaFree(d_costs);
    cudaFree(d_target_pose);

    return best_joint_angles;
}
