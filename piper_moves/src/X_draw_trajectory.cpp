// Moveit
#include <cstddef>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
// Moveit X rviz
#include <moveit_visual_tools/moveit_visual_tools.h>
// ROS2
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
// Cpp
#include <cmath>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

const std::vector<std::vector<float>> grid_positions = {
    {0.245, 0.065, 0.213},  {0.20, 0.065, 0.213},  {0.155, 0.065, 0.213},
    {0.245, 0.020, 0.213},  {0.20, 0.020, 0.213},  {0.155, 0.020, 0.213},
    {0.245, -0.025, 0.213}, {0.20, -0.025, 0.213}, {0.155, -0.025, 0.213},
};

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
    // piper_orders subscriber
    piper_orders_subscriber_ =
        node_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/piper_move_orders", 10,
            std::bind(&PiperTrajectory::piper_orders_callback, this, _1));
    piper_orders_response_publisher_ =
        node_->create_publisher<std_msgs::msg::Bool>(
            "/piper_move_orders_response_", 10);

    // Exec
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

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

    // variables
    //  init current_order
    current_order_ = {0, 0};
  }

  void target_plan() {
    for (int i = 0; i < 10; i++) {
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
          piper_curr_pos_.pose.orientation.y,
          piper_curr_pos_.pose.orientation.z,
          piper_curr_pos_.pose.orientation.w);

      // pre-pose plan + execution
      prompt_("Press 'Next' in the RvizVisualToolsGui to go to the ordered "
              "pre-position (if any published in /piper_move_orders)");
      draw_title_("pre-pose");
      moveit_visual_tools_->trigger();
      // check piper current_order_ and go to this pos before all
      if (current_order_[0] >= 0 && current_order_[0] < 10) {
        int position_index = current_order_[0];
        setup_goal_pose_target(
            static_cast<float>(grid_positions[position_index][0]),
            static_cast<float>(grid_positions[position_index][1]),
            static_cast<float>(grid_positions[position_index][2]), 0.000034,
            1.000000, -0.000004, 0.000027);
        piper_interface_->plan(simple_plan_);
        piper_interface_->execute(simple_plan_);
      }

      // create plan
      prompt_("Press 'Next' in the RvizVisualToolsGui window to plan");
      draw_title_("Planning");
      moveit_visual_tools_->trigger();
      if (current_order_[1] == 0) {
        plan_fraction_robot_ = piper_interface_->computeCartesianPath(
            generate_circle_pose_targets(), end_effector_step_, jump_threshold_,
            cartesian_trajectory_plan_);
      } else if (current_order_[1] == 1) {
        plan_fraction_robot_ = piper_interface_->computeCartesianPath(
            generate_cross_pose_targets(), end_effector_step_, jump_threshold_,
            cartesian_trajectory_plan_);
      } else if (current_order_[1] == 2) {
        plan_fraction_robot_ = piper_interface_->computeCartesianPath(
            generate_grid_pose_targets(), end_effector_step_, jump_threshold_,
            cartesian_trajectory_plan_);
      }

      // Execute plan
      if (plan_fraction_robot_ >= 0.6) {
        draw_trajectory_tool_path_(cartesian_trajectory_plan_);
        moveit_visual_tools_->trigger();
        RCLCPP_INFO(logger_, "Valid Trajectory! Percent Valid: %f",
                    plan_fraction_robot_);
        prompt_("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title_("Executing");
        moveit_visual_tools_->trigger();
        // TOTG PART
        // Convert to RobotTrajectory object
        robot_trajectory::RobotTrajectory rt(piper_interface_->getRobotModel(),
                                             "arm");
        rt.setRobotTrajectoryMsg(*piper_interface_->getCurrentState(),
                                 cartesian_trajectory_plan_);

        // Time-parameterize with velocity/accel limits
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool success = totg.computeTimeStamps(rt,
                                              0.05, // vel scale
                                              0.05  // accel scale
        );
        if (success) {
          rt.getRobotTrajectoryMsg(cartesian_trajectory_plan_);
          piper_interface_->execute(cartesian_trajectory_plan_);
          RCLCPP_INFO(logger_, "Successfully executed the order!");
        } else {
          RCLCPP_ERROR(logger_, "Execution failed, TOTG problem!");
        }
      } else {
        draw_title_("Planning-Failed!");
        moveit_visual_tools_->trigger();
        RCLCPP_ERROR(logger_, "Planning failed!");
      }

      // Come back to retract-pose !
      prompt_(
          "Press 'Next' in the RvizVisualToolsGui to go to the RETRACT Pose");
      draw_title_("pre-pose-AGAIN");
      moveit_visual_tools_->trigger();
      // check piper current_order_ and go to this pos before all
      if (current_order_[0] >= 0 && current_order_[0] < 10) {
        setup_goal_pose_target(0.212, -0.108, 0.214, 0.000034, 1.000000,
                               -0.000004, 0.000027);
        piper_interface_->plan(simple_plan_);
        piper_interface_->execute(simple_plan_);
        current_order_response_.data = true;
        piper_orders_response_publisher_->publish(current_order_response_);
      }
    }
  }

  std::vector<geometry_msgs::msg::Pose>
  generate_circle_pose_targets(double retract_amount = 0.01,
                               double radius = 0.015,
                               double nb_waypoints = 200) {
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

    geometry_msgs::msg::Pose step_goal;
    // circle gen
    for (int j = 0; j < nb_waypoints; j++) {
      step_goal.orientation = piper_curr_pos_.pose.orientation;
      step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;

      double current_angle = 2 * M_PI * j / nb_waypoints;

      double x_offset = radius * cos(current_angle);
      double y_offset = radius * sin(current_angle);

      step_goal.position.x = piper_curr_pos_.pose.position.x + x_offset;
      step_goal.position.y = piper_curr_pos_.pose.position.y + y_offset;

      targets.push_back(step_goal);
    }

    // retract movement
    step_goal.position.z = piper_curr_pos_.pose.position.z;
    targets.push_back(step_goal);

    return targets;
  }

  std::vector<geometry_msgs::msg::Pose>
  generate_cross_pose_targets(double retract_amount = 0.01,
                              double square_size = 0.03) {
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

    // cross figure
    geometry_msgs::msg::Pose step_goal;
    step_goal.orientation = piper_curr_pos_.pose.orientation;
    step_goal.position.z = piper_curr_pos_.pose.position.z;

    // starting point: upper left corner
    step_goal.position.x = piper_curr_pos_.pose.position.x + square_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y + square_size / 2;
    targets.push_back(step_goal);
    // engage movement (down z)
    step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;
    targets.push_back(step_goal);
    // opposite side
    step_goal.position.x = piper_curr_pos_.pose.position.x - square_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y - square_size / 2;
    targets.push_back(step_goal);

    // retract movement
    step_goal.position.z = piper_curr_pos_.pose.position.z + retract_amount;
    targets.push_back(step_goal);

    // upper right corner
    step_goal.position.x = piper_curr_pos_.pose.position.x + square_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y - square_size / 2;
    targets.push_back(step_goal);
    // engage movement (down z)
    step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;
    targets.push_back(step_goal);
    // opposite side of upper right corner
    step_goal.position.x = piper_curr_pos_.pose.position.x - square_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y + square_size / 2;
    targets.push_back(step_goal);

    // final retract movement
    step_goal.position.z = piper_curr_pos_.pose.position.z + retract_amount;
    targets.push_back(step_goal);

    return targets;
  }

  std::vector<geometry_msgs::msg::Pose>
  generate_grid_pose_targets(double retract_amount = 0.01,
                             double grid_size = 0.12) {
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

    // Constant orientation
    geometry_msgs::msg::Pose step_goal;
    step_goal.orientation = piper_curr_pos_.pose.orientation;
    step_goal.position.z = piper_curr_pos_.pose.position.z;

    // bottom horizontal line (your existing one)
    step_goal.position.x = piper_curr_pos_.pose.position.x - grid_size / 6;
    step_goal.position.y = piper_curr_pos_.pose.position.y + grid_size / 2;
    targets.push_back(step_goal);
    // (down z)
    step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;
    targets.push_back(step_goal);
    // trace line
    step_goal.position.x = piper_curr_pos_.pose.position.x - grid_size / 6;
    step_goal.position.y = piper_curr_pos_.pose.position.y - grid_size / 2;
    targets.push_back(step_goal);
    // (up z)
    step_goal.position.z = piper_curr_pos_.pose.position.z + retract_amount;
    targets.push_back(step_goal);

    // top horizontal line
    step_goal.position.z = piper_curr_pos_.pose.position.z;
    step_goal.position.x = piper_curr_pos_.pose.position.x + grid_size / 6;
    step_goal.position.y = piper_curr_pos_.pose.position.y + grid_size / 2;
    targets.push_back(step_goal);
    // (down z)
    step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;
    targets.push_back(step_goal);
    // trace line
    step_goal.position.x = piper_curr_pos_.pose.position.x + grid_size / 6;
    step_goal.position.y = piper_curr_pos_.pose.position.y - grid_size / 2;
    targets.push_back(step_goal);
    // (up z)
    step_goal.position.z = piper_curr_pos_.pose.position.z + retract_amount;
    targets.push_back(step_goal);

    // right vertical line
    step_goal.position.z = piper_curr_pos_.pose.position.z;
    step_goal.position.x = piper_curr_pos_.pose.position.x + grid_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y - grid_size / 6;
    targets.push_back(step_goal);
    // (down z)
    step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;
    targets.push_back(step_goal);
    // trace line
    step_goal.position.x = piper_curr_pos_.pose.position.x - grid_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y - grid_size / 6;
    targets.push_back(step_goal);
    // (up z)
    step_goal.position.z = piper_curr_pos_.pose.position.z + retract_amount;
    targets.push_back(step_goal);

    // left vertical line
    step_goal.position.z = piper_curr_pos_.pose.position.z;
    step_goal.position.x = piper_curr_pos_.pose.position.x - grid_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y + grid_size / 6;
    targets.push_back(step_goal);
    // (down z)
    step_goal.position.z = piper_curr_pos_.pose.position.z - retract_amount;
    targets.push_back(step_goal);
    // trace line
    step_goal.position.x = piper_curr_pos_.pose.position.x + grid_size / 2;
    step_goal.position.y = piper_curr_pos_.pose.position.y + grid_size / 6;
    targets.push_back(step_goal);
    // (up z)
    step_goal.position.z = piper_curr_pos_.pose.position.z + retract_amount;
    targets.push_back(step_goal);

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

  void
  piper_orders_callback(const std_msgs::msg::Int32MultiArray::SharedPtr order) {
    RCLCPP_INFO(logger_, "Received order!: [%d, %d]", order->data[0],
                order->data[1]);
    current_order_[0] = order->data[0];
    current_order_[1] = order->data[1];
  }

  // getters
  std::thread &spinner() { return *spinner_; }

private:
  // Node, Pub/Sub and executor
  std::shared_ptr<MoveGroupInterface> piper_interface_;
  rclcpp::Logger logger_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      piper_orders_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      piper_orders_response_publisher_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
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
  // piper orders
  std::array<int32_t, 2> current_order_;
  MoveGroupInterface::Plan simple_plan_;
  std_msgs::msg::Bool current_order_response_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto piper_moves = std::make_shared<PiperTrajectory>();
  piper_moves->target_plan();
  rclcpp::shutdown();
  piper_moves->spinner().join();
  return 0;
}