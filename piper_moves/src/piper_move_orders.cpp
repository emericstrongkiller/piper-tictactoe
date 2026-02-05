#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
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

    // game start topic sub
    game_start_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/game_start", 10,
            std::bind(&PiperMoveOrders::game_start_subscriber_callback, this,
                      _1));

    // new game topic sub
    new_game_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/new_game", 10,
        std::bind(&PiperMoveOrders::new_game_subscriber_callback, this, _1));

    // first piper order
    current_order_ = std::make_shared<std_msgs::msg::Int32MultiArray>();
    current_game_board_squares_ =
        std::make_shared<std_msgs::msg::Int32MultiArray>();
    current_game_board_squares_->data.resize(9, 0);

    // other inits
    first_turn_ = true;
  }

  void
  order_response_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
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
      RCLCPP_DEBUG(this->get_logger(), "msg->data[i]: %d", msg->data[i]);
      if (static_cast<int>(i) < 2) {
        current_order_->data.push_back(msg->data[i]);
      } else {
        current_game_board_squares_->data.push_back(msg->data[i]);
      }
    }
    // convert circle from 2 to 0 because im a dumbasss
    if (current_order_->data[1] == 2) {
      current_order_->data[1] = 0;
    }
    if (current_order_->data[1] != -1 && current_order_->data[0] != -1) {
      RCLCPP_INFO(this->get_logger(), "Tempo...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      RCLCPP_INFO(this->get_logger(), "MOVING TO POS: %d",
                  current_order_->data[0]);
      order_publisher_->publish(*current_order_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Invalid best move! Game ended ?");
    }
  }

  void game_start_subscriber_callback(
      const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Napping...");
    std::this_thread::sleep_for(std::chrono::milliseconds(40000));
    RCLCPP_INFO(this->get_logger(), "UP AND READY");
    game_infos_.data = msg->data;
    // either start game or start asking camera to query game board depending on
    // shape chosen for the robot
    RCLCPP_INFO(this->get_logger(), "ROBOT SHAPE FOR THIS GAME: %d",
                msg->data[0]);
    if (first_turn_) {
      // circle => check human first move with camera
      if (game_infos_.data[0] == 2) {
        // send camera order to query board grid
        RCLCPP_INFO(this->get_logger(),
                    "Asking camera to query the board game grid..");
        camera_current_order_.data = true;
        camera_order_publisher_->publish(camera_current_order_);
      }
      // cross => do the first move, it will then trigger the rest of the
      // process !
      else if (game_infos_.data[0] == 1) {
        current_order_->data = {6, 1};
        order_publisher_->publish(*current_order_);
      }
      first_turn_ = false;
    }
  }

  void new_game_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // init first turn   so that we start a nwe game cleeean
    RCLCPP_INFO(this->get_logger(), "Ready to start new Game !");
    first_turn_ = msg->data;
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
  // start game
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      game_start_subscriber_;
  // shape infos
  std_msgs::msg::Int32MultiArray game_infos_;
  bool first_turn_;

  // new game sub
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr new_game_subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiperMoveOrders>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}