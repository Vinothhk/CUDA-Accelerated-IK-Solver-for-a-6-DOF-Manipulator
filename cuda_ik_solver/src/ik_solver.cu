#include <cmath>
#include <vector>
#include "cuda_ik_solver/ik_solver.hpp"

#define DOF 6
#define NUM_CANDIDATES 512
#define DAMPING_FACTOR 0.1f

__device__ __constant__ float d_joint_lower[DOF];
__device__ __constant__ float d_joint_upper[DOF];
__device__ __constant__ float d_previous_solution[DOF];

namespace {

__device__ float bounded_noise(float range) {
    unsigned int seed = threadIdx.x + blockIdx.x * blockDim.x;
    seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    return (seed / (float)0x7fffffff) * 2.0f * range - range;
}

__device__ void forward_kinematics(const float* joint_angles, float* ee_pose, float* ee_quat) {
    float a[DOF] = {0.0f, -0.425f, -0.3922f, 0.0f, 0.0f, 0.0f};
    float d[DOF] = {0.1625f, 0.0f, 0.0f, 0.1333f, 0.0997f, 0.0996f};
    float alpha[DOF] = {-M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, 0.0f};

    float T[4][4] = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};

    for (int i = 0; i < DOF; ++i) {
        float th = joint_angles[i];
        float ct = cosf(th), st = sinf(th);
        float ca = cosf(alpha[i]), sa = sinf(alpha[i]);

        float A[4][4] = {
            {ct, -st*ca, st*sa, a[i]*ct},
            {st, ct*ca, -ct*sa, a[i]*st},
            {0, sa, ca, d[i]},
            {0, 0, 0, 1.0f}
        };

        float result[4][4] = {{0}};
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                for (int k = 0; k < 4; ++k) {
                    result[r][c] += T[r][k] * A[k][c];
                }
            }
        }
        memcpy(T, result, sizeof(float)*16);
    }

    ee_pose[0] = T[0][3];
    ee_pose[1] = T[1][3];
    ee_pose[2] = T[2][3];

    // Convert 3x3 rotation matrix (top-left of T) to quaternion
    float trace = T[0][0] + T[1][1] + T[2][2];
    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        ee_quat[3] = 0.25f * s;
        ee_quat[0] = (T[2][1] - T[1][2]) / s;
        ee_quat[1] = (T[0][2] - T[2][0]) / s;
        ee_quat[2] = (T[1][0] - T[0][1]) / s;
    } else {
        // fallback branches (omitted here for brevity)
    }

}

__device__ float quaternion_distance(const float* q1, const float* q2) {
    float dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    return 1.0f - (dot * dot);  // 0 = identical, 1 = opposite
}

__device__ float compute_cost(const float* joint_angles, const float* target_pose) {
    float ee_pose[3];
    float ee_quat[4];
    forward_kinematics(joint_angles, ee_pose, ee_quat);

    // Position error
    float dx = target_pose[0] - ee_pose[0];
    float dy = target_pose[1] - ee_pose[1];
    float dz = target_pose[2] - ee_pose[2];
    float pos_cost = dx*dx + dy*dy + dz*dz;

    // Orientation error
    float ori_cost = quaternion_distance(&target_pose[3], ee_quat);

    return pos_cost + 0.1f * ori_cost;  // Tune weight
}
    // forward_kinematics(joint_angles, ee_pose);

    // float dx = target_pose[0] - ee_pose[0];
    // float dy = target_pose[1] - ee_pose[1];
    // float dz = target_pose[2] - ee_pose[2];

    // return dx*dx + dy*dy + dz*dz;

__global__ void evaluate_ik_solutions(float* candidate_angles, float* costs, 
                                    const float* target_pose, int num_candidates) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= num_candidates) return;

    float* joints = &candidate_angles[idx * DOF];
    
    if (idx < num_candidates/2) {
        for (int j = 0; j < DOF; ++j) {
            float noise = bounded_noise(0.2f);
            joints[j] = d_previous_solution[j] + noise;
            joints[j] = fmaxf(d_joint_lower[j], fminf(joints[j], d_joint_upper[j]));
        }
    } else {
        for (int j = 0; j < DOF; ++j) {
            float noise = bounded_noise(0.5f);
            joints[j] = d_previous_solution[j] + noise;
            joints[j] = fmaxf(d_joint_lower[j], fminf(joints[j], d_joint_upper[j]));
        }
    }
    
    costs[idx] = compute_cost(joints, target_pose);
}

} // namespace

std::vector<double> cuda_ik_solver::solveIKCUDA(const geometry_msgs::msg::Pose& target_pose, 
                                              const std::vector<double>& current_joint_angles) {
    // Host memory
    float h_candidates[NUM_CANDIDATES * DOF];
    float h_costs[NUM_CANDIDATES];
    float h_target_pose[7] = {
        static_cast<float>(target_pose.position.x),
        static_cast<float>(target_pose.position.y),
        static_cast<float>(target_pose.position.z),
        static_cast<float>(target_pose.orientation.x),
        static_cast<float>(target_pose.orientation.y),
        static_cast<float>(target_pose.orientation.z),
        static_cast<float>(target_pose.orientation.w)
    };

    // Device memory
    float *d_candidates = nullptr;
    float *d_costs = nullptr;
    float *d_target_pose = nullptr;
    
    cudaMalloc(&d_candidates, sizeof(float) * NUM_CANDIDATES * DOF);
    cudaMalloc(&d_costs, sizeof(float) * NUM_CANDIDATES);
    cudaMalloc(&d_target_pose, sizeof(float) * 3);

    // Set joint limits and previous solution
    static const float joint_limits[DOF][2] = {
        {-2.8973f, 2.8973f}, {-1.7628f, 1.7628f}, 
        {-2.8973f, 2.8973f}, {-3.0718f, -0.0698f},
        {-2.8973f, 2.8973f}, {-0.0175f, 3.7525f}
    };
        
    float lower[DOF], upper[DOF];
    for (int i = 0; i < DOF; ++i) {
        lower[i] = joint_limits[i][0];
        upper[i] = joint_limits[i][1];
    }
    cudaMemcpyToSymbol(d_joint_lower, lower, sizeof(float) * DOF);
    cudaMemcpyToSymbol(d_joint_upper, upper, sizeof(float) * DOF);

    // cudaMemcpyToSymbol(d_joint_lower, &joint_limits[0][0], sizeof(float) * DOF);
    // cudaMemcpyToSymbol(d_joint_upper, &joint_limits[0][1], sizeof(float) * DOF);
    
    float current_angles_float[DOF];
    for (int i = 0; i < DOF; ++i) {
        current_angles_float[i] = static_cast<float>(current_joint_angles[i]);
    }
    cudaMemcpyToSymbol(d_previous_solution, current_angles_float, sizeof(float) * DOF);

    // Copy target pose to device
    cudaMemcpy(d_target_pose, h_target_pose, sizeof(float) * 3, cudaMemcpyHostToDevice);

    // Launch kernel
    const int BLOCK_SIZE = 256;
    const int GRID_SIZE = (NUM_CANDIDATES + BLOCK_SIZE - 1) / BLOCK_SIZE;
    evaluate_ik_solutions<<<GRID_SIZE, BLOCK_SIZE>>>(d_candidates, d_costs, d_target_pose, NUM_CANDIDATES);

    // Get results
    cudaMemcpy(h_costs, d_costs, sizeof(float) * NUM_CANDIDATES, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_candidates, d_candidates, sizeof(float) * NUM_CANDIDATES * DOF, cudaMemcpyDeviceToHost);

    // Find best solution
    int best_idx = 0;
    for (int i = 1; i < NUM_CANDIDATES; ++i) {
        if (h_costs[i] < h_costs[best_idx]) {
            best_idx = i;
        }
    }

    // Cleanup
    cudaFree(d_candidates);
    cudaFree(d_costs);
    cudaFree(d_target_pose);

    // Return solution
    std::vector<double> result(DOF);
    for (int j = 0; j < DOF; ++j) {
        result[j] = static_cast<double>(h_candidates[best_idx * DOF + j]);
    }
    return result;
}