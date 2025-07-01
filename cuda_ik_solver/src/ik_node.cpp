#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "cuda_ik_solver/ik_solver.hpp"
#include <cmath>
#include <vector>

class IKNode : public rclcpp::Node {
public:
  IKNode() : Node("ik_node") {
    RCLCPP_INFO(this->get_logger(), "IKNode started. Waiting for EE goals and joint states...");

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_goal", 10, std::bind(&IKNode::pose_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&IKNode::joint_state_callback, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  std::vector<double> current_joint_angles_;
  bool has_joint_state_ = false;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6) {
      current_joint_angles_.assign(msg->position.begin(), msg->position.begin() + 6);
      has_joint_state_ = true;
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!has_joint_state_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for joint state before solving IK.");
      return;
    }

    // Ensure pose is in base frame
    // if (msg->header.frame_id != "panda_link0" && msg->header.frame_id != "base_link") {
    //   // RCLCPP_INFO()
    //   RCLCPP_WARN(this->get_logger(), "Expected pose in 'panda_link0' or 'base_link'. Got '%s'", msg->header.frame_id.c_str());
    //   return;
    // }

    RCLCPP_INFO(this->get_logger(), "EE Goal received: [x=%.3f, y=%.3f, z=%.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    auto joint_angles = cuda_ik_solver::solveIKCUDA(msg->pose, current_joint_angles_);

    // Check if change is negligible
    double delta_sum = 0.0;
    for (size_t i = 0; i < joint_angles.size(); ++i)
      delta_sum += std::abs(joint_angles[i] - current_joint_angles_[i]);

    if (delta_sum < 1e-3) {
      RCLCPP_INFO(this->get_logger(), "Goal is already reached. Skipping command.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing new joint trajectory...");

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_angles;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    traj.points.push_back(point);
    traj_pub_->publish(traj);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include "cuda_ik_solver/ik_solver.hpp"

// class IKNode : public rclcpp::Node {
// public:
//   IKNode() : Node("ik_node") {
//     RCLCPP_INFO(this->get_logger(), "IKNode initialized. Subscribing to /ee_goal");

//     pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/ee_goal", 10,
//       std::bind(&IKNode::pose_callback, this, std::placeholders::_1));

//     traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//       "/joint_trajectory_controller/joint_trajectory", 10);
//   }

// private:
//   void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), "Received EE goal: [x=%.3f, y=%.3f, z=%.3f]",
//                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

//     std::vector<double> joint_angles = cuda_ik_solver::solveIKCUDA(msg->pose);

//     RCLCPP_INFO(this->get_logger(), "Computed joint angles:");
//     for (size_t i = 0; i < joint_angles.size(); ++i) {
//       RCLCPP_INFO(this->get_logger(), "  joint[%zu] = %.3f", i, joint_angles[i]);
//     }

//     trajectory_msgs::msg::JointTrajectory traj;
//     traj.joint_names = {
//       "shoulder_pan_joint",
//       "shoulder_lift_joint",
//       "elbow_joint",
//       "wrist_1_joint",
//       "wrist_2_joint",
//       "wrist_3_joint"
//     };

//     trajectory_msgs::msg::JointTrajectoryPoint point;
//     point.positions = joint_angles;
//     point.time_from_start = rclcpp::Duration::from_seconds(2.0);

//     traj.points.push_back(point);
//     traj_pub_->publish(traj);

//     RCLCPP_INFO(this->get_logger(), "Published joint trajectory to controller.");
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<IKNode>());
//   rclcpp::shutdown();
//   return 0;
// }
