#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <cstddef>
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
        "/piper_move_orders", 1);
    order_response_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/piper_move_orders_response_", 10,
        std::bind(&PiperMoveOrders::order_response_subscriber_callback, this,
                  _1));

    // perception orders / feedback
    camera_order_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("/take_pic_order", 10);
    min_max_tictactoe_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/best_move_min_max", 10,
            std::bind(&PiperMoveOrders::min_max_tictactoe_subscriber_callback,
                      this, _1));

    // first piper order
    current_order_ = std::make_shared<std_msgs::msg::Int32MultiArray>();
    current_game_board_squares_ =
        std::make_shared<std_msgs::msg::Int32MultiArray>();
    current_game_board_squares_->data.resize(9, 0);
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

      //// populate current_game_board_squares_
      // current_game_board_squares_->data.resize(9);
      // for (int i = 0; i < 9; i++) {
      //   current_game_board_squares_->data[i] = 0;
      // }
      // current_game_board_squares_->data[current_order_->data[0]] = 2;

      for (size_t i = 0; i < current_game_board_squares_->data.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "board_game N-%d: %d",
                    static_cast<int>(i),
                    static_cast<int>(current_game_board_squares_->data[i]));
      }

      // Spawn a shape at a random non-filled grid square
      RCLCPP_INFO(this->get_logger(),
                  "Gonna spawn a cross for the HUMAN Player now");
      for (size_t i = 0; i < current_game_board_squares_->data.size(); i++) {
        if (static_cast<int>(current_game_board_squares_->data[i]) == 0) {
          current_spawn_order_ =
              "ros2 run gazebo_ros spawn_entity.py -entity " + shape_names[1] +
              std::to_string(i) +
              " -file /home/user/ros2_ws/src/piper_ros/models/" +
              shape_names[1] + "/model.sdf" + " -x " +
              std::to_string(grid_shapes_positions[i][0]) + " -y " +
              std::to_string(grid_shapes_positions[i][1]) + " -z " +
              std::to_string(grid_shapes_positions[i][2]);

          std::system(current_spawn_order_.c_str());
          break;
        }
      }

      RCLCPP_INFO(this->get_logger(), "SLEEP 10");
      rclcpp::sleep_for(std::chrono::seconds(10));
      RCLCPP_INFO(this->get_logger(), "DONE SLEEPING");

      // send camera order to query board grid
      RCLCPP_INFO(this->get_logger(),
                  "Asking camera to query the board game grid..");
      camera_current_order_.data = true;
      camera_order_publisher_->publish(camera_current_order_);

    } else {
      RCLCPP_INFO(this->get_logger(), "ERROR executing the order..");
    }
  }

  void min_max_tictactoe_subscriber_callback(
      const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // CLEAR CURRENT ORDER BEFORE ALL
    current_order_->data.clear();
    current_game_board_squares_->data.clear();
    // get the best move from MinMax TicTacToe
    RCLCPP_INFO(
        this->get_logger(),
        "Received best_move from minMaxTicTacToe, about to send move_order..");
    for (size_t i = 0; i < msg->data.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "msg->data[i]: %d", msg->data[i]);
      if (static_cast<int>(i) < 2) {
        current_order_->data.push_back(msg->data[i]);
      } else {
        current_game_board_squares_->data.push_back(msg->data[i]);
      }
    }
    order_publisher_->publish(*current_order_);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr order_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      order_response_subscriber_;
  std_msgs::msg::Int32MultiArray::SharedPtr current_order_;
  std_msgs::msg::Int32MultiArray::SharedPtr current_game_board_squares_;
  std::string current_spawn_order_;
  // perception + MinMax treatment
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_order_publisher_;
  std_msgs::msg::Bool camera_current_order_;
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