// CUDA IK Solver for Franka Emika Panda Arm (6 DOF) with Bias Sampling & Frame-Aware Computation
#include <cmath>
#include <vector>
#include <iostream>
#include <geometry_msgs/msg/pose.hpp>
#include "cuda_ik_solver/ik_solver.hpp"

#define DOF 6
#define NUM_CANDIDATES 256

// Device constant memory for joint limits
__device__ __constant__ float d_joint_lower[DOF];
__device__ __constant__ float d_joint_upper[DOF];

// ---------------- CUDA Device Code ----------------
__device__ void forward_kinematics(const float* joint_angles, float* ee_pose) {
    float a[DOF] = {0.0f, 0.0f, 0.0f, 0.0825f, -0.0825f, 0.0f};
    float d[DOF] = {0.333f, 0.0f, 0.316f, 0.0f, 0.384f, 0.0f};
    float alpha[DOF] = {-M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2};

    float T[4][4] = {{1, 0, 0, 0},
                     {0, 1, 0, 0},
                     {0, 0, 1, 0},
                     {0, 0, 0, 1}};

    for (int i = 0; i < DOF; ++i) {
        float theta = joint_angles[i];
        float ct = cosf(theta), st = sinf(theta);
        float ca = cosf(alpha[i]), sa = sinf(alpha[i]);

        float A[4][4] = {
            {ct, -st * ca, st * sa, a[i] * ct},
            {st, ct * ca, -ct * sa, a[i] * st},
            {0, sa, ca, d[i]},
            {0, 0, 0, 1}
        };

        float result[4][4];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                result[r][c] = 0;
                for (int k = 0; k < 4; ++k)
                    result[r][c] += T[r][k] * A[k][c];
            }

        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                T[r][c] = result[r][c];
    }

    ee_pose[0] = T[0][3];
    ee_pose[1] = T[1][3];
    ee_pose[2] = T[2][3];
}

__device__ float compute_cost(const float* joint_angles, const float* target_pose) {
    float ee_pose[3];
    forward_kinematics(joint_angles, ee_pose);

    float cost = 0.0f;
    for (int i = 0; i < 3; ++i)
        cost += (ee_pose[i] - target_pose[i]) * (ee_pose[i] - target_pose[i]);

    if (ee_pose[2] < 0.1f) cost += 1000.0f;  // Avoid floor collision

    return cost;
}

__global__ void evaluate_ik_solutions(float* candidate_angles, float* costs, float* target_pose, int num_candidates) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < num_candidates) {
        float* joint_angles = &candidate_angles[idx * DOF];
        costs[idx] = compute_cost(joint_angles, target_pose);
    }
}

// ---------------- Host Code ----------------
std::vector<double> cuda_ik_solver::solveIKCUDA(const geometry_msgs::msg::Pose& target_pose, const std::vector<double>& current_joint_angles) {
    float h_candidates[NUM_CANDIDATES * DOF];
    float h_costs[NUM_CANDIDATES];
    float h_target_pose[6] = {
        static_cast<float>(target_pose.position.x),
        static_cast<float>(target_pose.position.y),
        static_cast<float>(target_pose.position.z),
        0.0f, 0.0f, 0.0f
    };

    float joint_lower[DOF] = {-2.8973f, -1.7628f, -2.8973f, -3.0718f, -2.8973f, -0.0175f};
    float joint_upper[DOF] = { 2.8973f,  1.7628f,  2.8973f, -0.0698f,  2.8973f,  3.7525f};

    cudaMemcpyToSymbol(d_joint_lower, joint_lower, sizeof(float) * DOF);
    cudaMemcpyToSymbol(d_joint_upper, joint_upper, sizeof(float) * DOF);

    // Biased sampling near current joint config with noise
    for (int i = 0; i < NUM_CANDIDATES; ++i) {
        for (int j = 0; j < DOF; ++j) {
            float noise = ((rand() % 2000) / 1000.0f - 1.0f) * 0.1f; // ±0.1 rad noise
            float center = static_cast<float>(current_joint_angles[j]);
            float sample = center + noise;
            sample = fminf(fmaxf(sample, joint_lower[j]), joint_upper[j]);
            h_candidates[i * DOF + j] = sample;
        }
    }

    float *d_candidates, *d_costs, *d_target_pose;
    cudaMalloc(&d_candidates, sizeof(float) * NUM_CANDIDATES * DOF);
    cudaMalloc(&d_costs, sizeof(float) * NUM_CANDIDATES);
    cudaMalloc(&d_target_pose, sizeof(float) * 6);

    cudaMemcpy(d_candidates, h_candidates, sizeof(float) * NUM_CANDIDATES * DOF, cudaMemcpyHostToDevice);
    cudaMemcpy(d_target_pose, h_target_pose, sizeof(float) * 6, cudaMemcpyHostToDevice);

    evaluate_ik_solutions<<<(NUM_CANDIDATES + 255)/256, 256>>>(d_candidates, d_costs, d_target_pose, NUM_CANDIDATES);

    cudaMemcpy(h_costs, d_costs, sizeof(float) * NUM_CANDIDATES, cudaMemcpyDeviceToHost);

    int best_idx = 0;
    for (int i = 1; i < NUM_CANDIDATES; ++i) {
        if (h_costs[i] < h_costs[best_idx]) best_idx = i;
    }

    std::vector<double> best_joint_angles(DOF);
    for (int j = 0; j < DOF; ++j)
        best_joint_angles[j] = h_candidates[best_idx * DOF + j];

    cudaFree(d_candidates);
    cudaFree(d_costs);
    cudaFree(d_target_pose);

    return best_joint_angles;
}




// // CUDA IK Solver for a 6-DOF Arm (Simplified, FK-based optimization)
// #include <cmath>
// #include <vector>
// #include <iostream>
// #include <geometry_msgs/msg/pose.hpp>
// #include "cuda_ik_solver/ik_solver.hpp"

// #define DOF 6
// #define NUM_CANDIDATES 256

// // ---------------- CUDA Device Code ----------------
// __device__ void forward_kinematics(const float* joint_angles, float* ee_pose) {
//     // DH parameters for Franka Emika Panda Arm (first 6 joints)
//     float a[DOF] = {0.0f, 0.0f, 0.0f, 0.0825f, -0.0825f, 0.0f};
//     float d[DOF] = {0.333f, 0.0f, 0.316f, 0.0f, 0.384f, 0.0f};
//     float alpha[DOF] = {-M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2};

//     float T[4][4] = {{1, 0, 0, 0},
//                      {0, 1, 0, 0},
//                      {0, 0, 1, 0},
//                      {0, 0, 0, 1}};

//     for (int i = 0; i < DOF; ++i) {
//         float theta = joint_angles[i];
//         float ct = cosf(theta), st = sinf(theta);
//         float ca = cosf(alpha[i]), sa = sinf(alpha[i]);

//         float A[4][4] = {
//             {ct, -st * ca, st * sa, a[i] * ct},
//             {st, ct * ca, -ct * sa, a[i] * st},
//             {0, sa, ca, d[i]},
//             {0, 0, 0, 1}
//         };

//         float result[4][4];
//         for (int r = 0; r < 4; ++r) {
//             for (int c = 0; c < 4; ++c) {
//                 result[r][c] = 0;
//                 for (int k = 0; k < 4; ++k) {
//                     result[r][c] += T[r][k] * A[k][c];
//                 }
//             }
//         }
//         for (int r = 0; r < 4; ++r)
//             for (int c = 0; c < 4; ++c)
//                 T[r][c] = result[r][c];
//     }

//     ee_pose[0] = T[0][3];  // x
//     ee_pose[1] = T[1][3];  // y
//     ee_pose[2] = T[2][3];  // z
//     // Orientation can be added here if needed
// }

// __device__ float compute_cost(const float* joint_angles, const float* target_pose) {
//     float ee_pose[6];
//     forward_kinematics(joint_angles, ee_pose);

//     float cost = 0.0f;
//     for (int i = 0; i < 3; ++i)  // Position only
//         cost += (ee_pose[i] - target_pose[i]) * (ee_pose[i] - target_pose[i]);

//     return cost;
// }

// __global__ void evaluate_ik_solutions(float* candidate_angles, float* costs, float* target_pose, int num_candidates) {
//     int idx = threadIdx.x + blockIdx.x * blockDim.x;
//     if (idx < num_candidates) {
//         float* joint_angles = &candidate_angles[idx * DOF];
//         costs[idx] = compute_cost(joint_angles, target_pose);
//     }
// }

// // ---------------- Host Code ----------------
// std::vector<double> cuda_ik_solver::solveIKCUDA(const geometry_msgs::msg::Pose& target_pose) {
//     float h_candidates[NUM_CANDIDATES * DOF];
//     float h_costs[NUM_CANDIDATES];
//     float h_target_pose[6] = {
//         static_cast<float>(target_pose.position.x),
//         static_cast<float>(target_pose.position.y),
//         static_cast<float>(target_pose.position.z),
//         0.0f, 0.0f, 0.0f  // Orientation skipped for now
//     };

//     float min_angle = -3.14f;
//     float max_angle = 3.14f;
//     for (int i = 0; i < NUM_CANDIDATES * DOF; ++i) {
//         h_candidates[i] = min_angle + (static_cast<float>(rand()) / RAND_MAX) * (max_angle - min_angle);
//     }

//     // Allocate device memory
//     float *d_candidates, *d_costs, *d_target_pose;
//     cudaMalloc(&d_candidates, sizeof(float) * NUM_CANDIDATES * DOF);
//     cudaMalloc(&d_costs, sizeof(float) * NUM_CANDIDATES);
//     cudaMalloc(&d_target_pose, sizeof(float) * 6);

//     cudaMemcpy(d_candidates, h_candidates, sizeof(float) * NUM_CANDIDATES * DOF, cudaMemcpyHostToDevice);
//     cudaMemcpy(d_target_pose, h_target_pose, sizeof(float) * 6, cudaMemcpyHostToDevice);

//     evaluate_ik_solutions<<<(NUM_CANDIDATES + 255)/256, 256>>>(d_candidates, d_costs, d_target_pose, NUM_CANDIDATES);

//     cudaMemcpy(h_costs, d_costs, sizeof(float) * NUM_CANDIDATES, cudaMemcpyDeviceToHost);

//     // Debug print first 5 costs
//     for (int i = 0; i < 5; ++i)
//         std::cout << "Cost[" << i << "] = " << h_costs[i] << std::endl;

//     int best_idx = 0;
//     for (int i = 1; i < NUM_CANDIDATES; ++i) {
//         if (h_costs[i] < h_costs[best_idx]) best_idx = i;
//     }

//     std::vector<double> best_joint_angles(DOF);
//     for (int j = 0; j < DOF; ++j) {
//         best_joint_angles[j] = h_candidates[best_idx * DOF + j];
//     }

//     cudaFree(d_candidates);
//     cudaFree(d_costs);
//     cudaFree(d_target_pose);

//     return best_joint_angles;
// }
