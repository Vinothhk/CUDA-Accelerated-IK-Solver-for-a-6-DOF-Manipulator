// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include "cuda_ik_solver/ik_solver.hpp"

// class IKNode : public rclcpp::Node {
// public:
//   IKNode() : Node("ik_node") {
//     pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/ee_goal", 10,
//       std::bind(&IKNode::pose_callback, this, std::placeholders::_1));

//     traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//       "/joint_trajectory_controller/joint_trajectory", 10);
//   }

// private:
//   void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     auto joint_angles = cuda_ik_solver::solveIKCUDA(msg->pose);

//     trajectory_msgs::msg::JointTrajectory traj;
//     traj.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

//     trajectory_msgs::msg::JointTrajectoryPoint point;
//     point.positions = joint_angles;
//     point.time_from_start = rclcpp::Duration::from_seconds(2.0);

//     traj.points.push_back(point);
//     traj_pub_->publish(traj);
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "cuda_ik_solver/ik_solver.hpp"

class IKNode : public rclcpp::Node {
public:
  IKNode() : Node("ik_node") {
    RCLCPP_INFO(this->get_logger(), "IKNode initialized. Subscribing to /ee_goal");

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_goal", 10,
      std::bind(&IKNode::pose_callback, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received EE goal: [x=%.3f, y=%.3f, z=%.3f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    std::vector<double> joint_angles = cuda_ik_solver::solveIKCUDA(msg->pose);

    RCLCPP_INFO(this->get_logger(), "Computed joint angles:");
    for (size_t i = 0; i < joint_angles.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  joint[%zu] = %.3f", i, joint_angles[i]);
    }

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

    RCLCPP_INFO(this->get_logger(), "Published joint trajectory to controller.");
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
