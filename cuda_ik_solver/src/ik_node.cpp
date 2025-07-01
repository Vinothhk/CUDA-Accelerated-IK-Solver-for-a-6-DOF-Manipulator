// // // =========================== CPP FILE: ik_node.cpp ===========================

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "cuda_ik_solver/ik_solver.hpp"

// #define BASE_FRAME "base_link"
// #define EE_FRAME "tool0"

// class IKNode : public rclcpp::Node {

// public:
//     IKNode() : Node("ik_node"), 
//               tf_buffer_(this->get_clock()),
//               tf_listener_(tf_buffer_) 
//     {
//         // 10. Improved initialization
//         pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/ee_goal", rclcpp::QoS(10).best_effort(),
//             std::bind(&IKNode::pose_callback, this, std::placeholders::_1));

//         joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//             "/joint_states", rclcpp::QoS(100).best_effort(),
//             std::bind(&IKNode::joint_state_callback, this, std::placeholders::_1));

//         traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//             "/joint_trajectory_controller/joint_trajectory", 
//             rclcpp::QoS(10).reliable());

//         // Pre-configure trajectory template
//         traj_template_.joint_names = {
//             "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
//             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
//         };

//         RCLCPP_INFO(this->get_logger(), "IKNode initialized with enhanced stability");
//     }

// private:
//     // 1. Enhanced member variables
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;

//     std::vector<double> current_joint_angles_;
//     std::vector<double> previous_target_angles_;  // For smoothing
//     bool has_joint_state_ = false;
//     rclcpp::Time last_command_time_;
//     trajectory_msgs::msg::JointTrajectory traj_template_;  // Reusable message

//     // 2. Improved joint state callback
//     void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
//         if (msg->position.size() >= 6) {
//             // Maintain joint order consistency
//             std::vector<double> temp_angles(6);
//             for(size_t i = 0; i < 6; ++i) {
//                 temp_angles[i] = msg->position[i];
//             }
            
//             // Simple low-pass filter (α=0.3) for smoother motion
//             if(has_joint_state_) {
//                 for(int i = 0; i < 6; ++i) {
//                     current_joint_angles_[i] = 0.7 * current_joint_angles_[i] + 0.3 * temp_angles[i];
//                 }
//             } else {
//                 current_joint_angles_ = temp_angles;
//                 has_joint_state_ = true;
//             }
//         }
//     }

//     // 3. Enhanced pose callback
//     void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         // Rate limiting (min 50ms between commands)
//         if((this->now() - last_command_time_).nanoseconds() < 50'000'000) {
//             return;
//         }

//         if (!has_joint_state_) {
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
//                                "Waiting for joint states...");
//             return;
//         }

//         // 4. More robust TF lookup
//         geometry_msgs::msg::TransformStamped tf_base_to_ee;
//         try {
//             tf_base_to_ee = tf_buffer_.lookupTransform(
//                 BASE_FRAME, EE_FRAME, 
//                 // msg->header.stamp,  // Use message timestamp
//                 this->now(),  // Use current time for stability
//                 rclcpp::Duration::from_seconds(0.1));
//         } catch (const tf2::TransformException &ex) {
//             RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
//                                 "TF error: %s", ex.what());
//             return;
//         }

//         // 5. Improved distance check
//         double dx = msg->pose.position.x - tf_base_to_ee.transform.translation.x;
//         double dy = msg->pose.position.y - tf_base_to_ee.transform.translation.y;
//         double dz = msg->pose.position.z - tf_base_to_ee.transform.translation.z;
//         double distance = dx*dx + dy*dy + dz*dz;  // Skip sqrt for efficiency

//         if (distance < 0.0001) {  // 1cm squared
//             RCLCPP_DEBUG(this->get_logger(), "Target reached (distance: %.3f m)", sqrt(distance));
//             return;
//         }

//         // 6. IK Solution with fallback
//         std::vector<double> target_angles;
//         try {
//             target_angles = cuda_ik_solver::solveIKCUDA(msg->pose, current_joint_angles_);
            
//             // Simple solution validation
//             for(double angle : target_angles) {
//                 if(!std::isfinite(angle)) throw std::runtime_error("Invalid IK solution");
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "IK failed: %s - Using previous target", e.what());
//             if(previous_target_angles_.empty()) return;
//             target_angles = previous_target_angles_;
//         }

//         // 7. Enhanced trajectory generation
//         auto traj = traj_template_;  // Reuse pre-configured message
//         traj.header.stamp = this->now();
        
//         // Dynamic step calculation based on distance
//         int steps = std::clamp(static_cast<int>(sqrt(distance) * 50), 5, 30);
//         double duration = std::clamp(sqrt(distance) * 0.5, 0.1, 2.0);

//         for (int s = 1; s <= steps; ++s) {
//             trajectory_msgs::msg::JointTrajectoryPoint point;
//             point.positions.resize(6);
            
//             // Cubic easing for smoother motion
//             double alpha = static_cast<double>(s)/steps;
//             alpha = alpha * alpha * (3.0 - 2.0 * alpha);
            
//             for (int i = 0; i < 6; ++i) {
//                 point.positions[i] = current_joint_angles_[i] + 
//                                     (target_angles[i] - current_joint_angles_[i]) * alpha;
//             }
            
//             point.time_from_start = rclcpp::Duration::from_seconds(duration * alpha);
//             traj.points.push_back(point);
//         }

//         // 8. Publish with validation
//         if(validate_trajectory(traj)) {
//             traj_pub_->publish(traj);
//             previous_target_angles_ = target_angles;
//             last_command_time_ = this->now();
//             RCLCPP_DEBUG(this->get_logger(), "Published trajectory (steps: %d, duration: %.2fs)", 
//                         steps, duration);
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Invalid trajectory generated");
//         }
//     }

//     // 9. Trajectory validation
//     bool validate_trajectory(const trajectory_msgs::msg::JointTrajectory& traj) {
//         if(traj.points.empty()) return false;
        
//         for(const auto& point : traj.points) {
//             if(point.positions.size() != 6) return false;
            
//             for(double pos : point.positions) {
//                 if(!std::isfinite(pos)) return false;
//             }
//         }
//         return true;
//     }


// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<IKNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cuda_ik_solver/ik_solver.hpp"

#define BASE_FRAME "base_link"
#define EE_FRAME "tool0"

class IKNode : public rclcpp::Node {
public:
    IKNode() : Node("ik_node"),
               tf_buffer_(this->get_clock()),
               tf_listener_(tf_buffer_)
    {
        // Initialize subscribers with default QoS
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ee_goal", 10,
            std::bind(&IKNode::pose_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&IKNode::joint_state_callback, this, std::placeholders::_1));

        // Publisher with default QoS
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // Pre-configure trajectory template
        traj_template_.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        // Diagnostic timer to check for joint states
        diagnostic_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (!has_joint_state_) {
                    RCLCPP_WARN(this->get_logger(), "Waiting for joint states on /joint_states...");
                }
            });

        RCLCPP_INFO(this->get_logger(), "IKNode initialized");
    }

private:
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;
    
    // TF2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // State variables
    std::vector<double> current_joint_angles_;
    std::vector<double> previous_target_angles_;
    bool has_joint_state_ = false;
    trajectory_msgs::msg::JointTrajectory traj_template_;
    std::mutex joints_mutex_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() >= 6) {
            std::lock_guard<std::mutex> lock(joints_mutex_);
            current_joint_angles_ = {
                msg->position[0], msg->position[1], msg->position[2],
                msg->position[3], msg->position[4], msg->position[5]
            };
            has_joint_state_ = true;
        }
    }
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!has_joint_state_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring pose command - joint states not available");
        return;
    }

    try {
        // Get current end-effector pose
        geometry_msgs::msg::TransformStamped tf_base_to_ee;
        try {
            tf_base_to_ee = tf_buffer_.lookupTransform(
                BASE_FRAME, 
                EE_FRAME,
                msg->header.stamp,  // Use message timestamp
                rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
            return;
        }

        // Calculate distance to target
        double dx = msg->pose.position.x - tf_base_to_ee.transform.translation.x;
        double dy = msg->pose.position.y - tf_base_to_ee.transform.translation.y;
        double dz = msg->pose.position.z - tf_base_to_ee.transform.translation.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        if (distance < 0.01) {  // 1cm threshold
            RCLCPP_DEBUG(this->get_logger(), "Target already reached");
            return;
        }

        // Solve IK
        std::vector<double> target_angles;
        try {
            target_angles = cuda_ik_solver::solveIKCUDA(msg->pose, current_joint_angles_);
            
            // Validate solution
            for(double angle : target_angles) {
                if(!std::isfinite(angle)) {
                    throw std::runtime_error("Non-finite joint angle in solution");
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IK failed: %s", e.what());
            if(previous_target_angles_.empty()) return;
            target_angles = previous_target_angles_;
        }

        // Generate trajectory
        auto traj = traj_template_;
        // traj.header.stamp = rclcpp::Time(0);  // Execute immediately
        // traj.header.stamp = this->get_clock()->now();  // ✅ safer

        traj.header.frame_id = BASE_FRAME;
        
        // Calculate duration based on distance
        double duration = std::clamp(distance * 2.0, 1.0, 5.0);  // 1-5 seconds
        RCLCPP_INFO(this->get_logger(), "Calculated duration: %.2f seconds", duration);
        // Start point (current position)
        trajectory_msgs::msg::JointTrajectoryPoint start_point;
        {
            std::lock_guard<std::mutex> lock(joints_mutex_);
            start_point.positions = current_joint_angles_;
        }
        start_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
        traj.points.push_back(start_point);

        // End point (target position)
        trajectory_msgs::msg::JointTrajectoryPoint end_point;
        end_point.positions = target_angles;
        
        // Calculate velocities (distance/time)
        std::vector<double> velocities(6);
        for (size_t i = 0; i < 6; i++) {
            velocities[i] = (target_angles[i] - current_joint_angles_[i]) / duration;
        }
        
        end_point.velocities = velocities;
        end_point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        end_point.time_from_start = rclcpp::Duration::from_seconds(duration);
        traj.points.push_back(end_point);

        // Publish trajectory
        traj_pub_->publish(traj);
        previous_target_angles_ = target_angles;
        RCLCPP_INFO(this->get_logger(), "Published trajectory to reach (%.3f, %.3f, %.3f)",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in pose callback: %s", e.what());
    }
}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}