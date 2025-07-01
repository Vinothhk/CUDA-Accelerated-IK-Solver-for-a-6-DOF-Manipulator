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

        // trajectory
        auto traj = traj_template_;
        // traj.header.stamp = rclcpp::Time(0);  // Execute immediately
        // traj.header.stamp = this->get_clock()->now();

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
        
        trajectory_msgs::msg::JointTrajectoryPoint end_point;
        end_point.positions = target_angles;
        
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