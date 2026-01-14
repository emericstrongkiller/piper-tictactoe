// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// Moveit X rviz
#include <moveit_visual_tools/moveit_visual_tools.h>
// ROS2
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
// Cpp
#include <cmath>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PiperTrajectory {
public:
  PiperTrajectory() : logger_(rclcpp::get_logger("piper_moves_logger")) {
    RCLCPP_INFO(logger_, "Initializing piper trajectory class..");
    RCLCPP_INFO(logger_, "RVIZ_MARKER_TOPIC: %s",
                rviz_visual_tools::RVIZ_MARKER_TOPIC.c_str());
    // Node
    node_ = std::make_shared<rclcpp::Node>(
        "X_draw_trajectory",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));
    // Exec
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor_->add_node(node_);
    spinner_ = std::make_shared<std::thread>([this]() { executor_->spin(); });
    // move_group interface
    piper_interface_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    piper_interface_->setEndEffectorLink("link6");

    // Construct and initialize MoveItVisualTools
    moveit_visual_tools_ =
        std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
            piper_interface_->getRobotModel());
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();

    // Create closures for visualization
    draw_title_ = [this](auto text) {
      auto const text_pose = [] {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 0.5; // Place text 1m above the base link
        return msg;
      }();
      moveit_visual_tools_->publishText(
          text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    };
    prompt_ = [this](auto text) { moveit_visual_tools_->prompt(text); };
    draw_trajectory_tool_path_ =
        [this, jmg = piper_interface_->getRobotModel()->getJointModelGroup(
                   "arm")](auto const trajectory) {
          moveit_visual_tools_->publishTrajectoryLine(trajectory, jmg);
        };
  }

  void init() {
    // piper curr infos
    RCLCPP_INFO(logger_, "Planning Frame: %s",
                piper_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(logger_, "End Effector Link: %s",
                piper_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(logger_, "Available Planning Groups:");
    std::vector<std::string> group_names =
        piper_interface_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(logger_, "Group %ld: %s", i, group_names[i].c_str());
    }

    // set start state of robot to current state
    piper_interface_->setStartStateToCurrentState();
  }

  void target_plan() {
    // piper_interface_->setPositionTarget(0.2, -0.1, 0.2, "link6");

    // setup_goal_pose_target(0.211855, 0.107601, 0.213981, -0.000077, 1.000000,
    //                      -0.000062, 0.000089);

    plan_fraction_robot_ = piper_interface_->computeCartesianPath(
        generate_circle_pose_targets(), end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);

    if (plan_fraction_robot_ >= 0.6) {
      RCLCPP_INFO(logger_,
                  "Valid Trajectory, about to execute it !\nPercent Valid: %f",
                  plan_fraction_robot_);
      piper_interface_->execute(cartesian_trajectory_plan_);
    } else {
      RCLCPP_INFO(logger_, "TRAJECTORY PLANNING FAILED\nPercent Valid: %f ",
                  plan_fraction_robot_);
    }

    // get piper Pose
    piper_curr_pos_ = piper_interface_->getCurrentPose();
    RCLCPP_INFO(logger_, "got EndEffector: %s",
                piper_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(
        logger_,
        "got piper Pose: \npiper_x: %f \npiper_y: %f \npiper_z: %f "
        "\npiper_ori_x: %f \npiper_ori_y: %f \npiper_ori_z: %f "
        "\npiper_ori_w: %f",
        piper_curr_pos_.pose.position.x, piper_curr_pos_.pose.position.y,
        piper_curr_pos_.pose.position.z, piper_curr_pos_.pose.orientation.x,
        piper_curr_pos_.pose.orientation.y, piper_curr_pos_.pose.orientation.z,
        piper_curr_pos_.pose.orientation.w);

    // create plan
    prompt_("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title_("Planning");
    moveit_visual_tools_->trigger();
    auto const [success, plan] = [this] {
      MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(piper_interface_->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute plan
    if (success) {
      draw_trajectory_tool_path_(plan.trajectory_);
      moveit_visual_tools_->trigger();
      prompt_("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title_("Executing");
      moveit_visual_tools_->trigger();
      piper_interface_->execute(plan);
    } else {
      draw_title_("Planning-Failed!");
      moveit_visual_tools_->trigger();
      RCLCPP_ERROR(logger_, "Planning failed!");
    }
  }

  std::vector<geometry_msgs::msg::Pose>
  generate_circle_pose_targets(double radius = 0.004,
                               double nb_waypoints = 100) {
    std::vector<geometry_msgs::msg::Pose> targets;
    piper_curr_pos_ = piper_interface_->getCurrentPose();
    RCLCPP_INFO(
        logger_,
        "Piper Init Pose: \npiper_x: %f \npiper_y: %f \npiper_z: %f "
        "\npiper_ori_x: %f \npiper_ori_y: %f \npiper_ori_z: %f "
        "\npiper_ori_w: %f",
        piper_curr_pos_.pose.position.x, piper_curr_pos_.pose.position.y,
        piper_curr_pos_.pose.position.z, piper_curr_pos_.pose.orientation.x,
        piper_curr_pos_.pose.orientation.y, piper_curr_pos_.pose.orientation.z,
        piper_curr_pos_.pose.orientation.w);

    // first half of the circle
    for (int j = 0; j < nb_waypoints; j++) {
      geometry_msgs::msg::Pose step_goal;
      step_goal.orientation = piper_curr_pos_.pose.orientation;
      step_goal.position.z = piper_curr_pos_.pose.position.z;

      double current_angle = 2 * M_PI * j / nb_waypoints;

      double x_offset = radius * cos(current_angle);
      double y_offset = radius * sin(current_angle);

      step_goal.position.x = piper_curr_pos_.pose.position.x + x_offset;
      step_goal.position.y = piper_curr_pos_.pose.position.y + y_offset;

      targets.push_back(step_goal);
      RCLCPP_INFO(logger_,
                  "[%d]  |  x:%f | y:%f |  z:%f | ori_x:%f | ori_y:%f | "
                  "ori_z:%f | ori_w:%f",
                  j, targets[j].position.x, targets[j].position.y,
                  targets[j].position.z, targets[j].orientation.x,
                  targets[j].orientation.y, targets[j].orientation.z,
                  targets[j].orientation.w);
    }

    return targets;
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    piper_interface_->setPoseTarget(target_pose_robot_);
  }

  // getters
  std::thread &spinner() { return *spinner_; }

private:
  // Node and executor
  std::shared_ptr<MoveGroupInterface> piper_interface_;
  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<std::thread> spinner_;
  // Robot infos
  geometry_msgs::msg::Pose target_pose_robot_;
  geometry_msgs::msg::PoseStamped piper_curr_pos_;
  // Cartesian paths params
  const double end_effector_step_ = 0.01;
  const double jump_threshold_ = 0.0;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
  double plan_fraction_robot_ = 0.0;
  // moveit visual tools
  std::function<void(const std::string &)> draw_title_;
  std::function<void(const std::string &)> prompt_;
  std::function<void(const moveit_msgs::msg::RobotTrajectory &)>
      draw_trajectory_tool_path_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto piper_moves = std::make_shared<PiperTrajectory>();
  piper_moves->target_plan();
  rclcpp::shutdown();
  piper_moves->spinner().join();
  return 0;
}