#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <memory>
#include <string>
#include <vector>

using namespace std::placeholders;

const std::vector<std::vector<float>> grid_shapes_positions = {
    {0.245, 0.065, 0.02},  {0.20, 0.065, 0.02},  {0.155, 0.065, 0.02},
    {0.245, 0.020, 0.02},  {0.20, 0.020, 0.02},  {0.155, 0.020, 0.02},
    {0.245, -0.025, 0.02}, {0.20, -0.025, 0.02}, {0.155, -0.025, 0.02},
};

const std::vector<std::string> shape_names = {"circle", "cross", "grid"};

class PiperMoveOrders : public rclcpp::Node {
public:
  PiperMoveOrders() : Node("piper_move_orders") {
    // piper orders
    order_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/piper_move_order", 1);
    order_response_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/piper_move_orders_response_", 10,
        std::bind(&PiperMoveOrders::order_response_subscriber_callback, this,
                  _1));

    // perception orders / feedback
    camera_order_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("/take_pic_order", 10);
    min_max_tictactoe_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/min_max_move_picker", 10,
            std::bind(&PiperMoveOrders::min_max_tictactoe_subscriber_callback,
                      this, _1));
    // first piper order
    current_order_ = std::make_shared<std_msgs::msg::Int32MultiArray>();
  }

  void
  order_response_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
      RCLCPP_INFO(this->get_logger(), "Order executed properly !");
      // Example order: ros2 run gazebo_ros spawn_entity.py -entity circle0
      // -file /home/user/ros2_ws/src/piper_ros/models/circle/model.sdf -x 0.245
      // -y 0.065 -z 0.02
      current_spawn_order_ =
          "ros2 run gazebo_ros spawn_entity.py -entity " +
          shape_names[current_order_->data[1]] +
          std::to_string(current_order_->data[0]) +
          " -file /home/user/ros2_ws/src/piper_ros/models/" +
          shape_names[current_order_->data[1]] + "/model.sdf" + " -x " +
          std::to_string(grid_shapes_positions[current_order_->data[0]][0]) +
          " -y " +
          std::to_string(grid_shapes_positions[current_order_->data[0]][1]) +
          " -z " +
          std::to_string(grid_shapes_positions[current_order_->data[0]][2]);

      std::system(current_spawn_order_.c_str());

    } else {
      RCLCPP_INFO(this->get_logger(), "ERROR executing the order..");
    }
  }

  void min_max_tictactoe_subscriber_callback(
      const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // get the best move from MinMax TicTacToe
    current_order_->data = msg->data;
    order_publisher_->publish(*current_order_);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr order_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      order_response_subscriber_;
  std_msgs::msg::Int32MultiArray::SharedPtr current_order_;
  std::string current_spawn_order_;
  // perception + MinMax treatment
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_order_publisher_;
  std_msgs::msg::Int32MultiArray current_game_board_squares_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      min_max_tictactoe_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiperMoveOrders>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}