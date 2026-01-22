#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

using namespace std::placeholders;

class MinMaxTicTacToe : public rclcpp::Node {
public:
  MinMaxTicTacToe() : Node("minmax_tic_tac_toe") {
    game_status_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/take_pic_order_response", 10,
            std::bind(&MinMaxTicTacToe::game_status_subscriber_callback_, this,
                      _1));

    best_move_publisher_ =
        this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/best_move_min_max", 10);

    // variables init
    current_game_board_2D_.resize(3, std::vector<int>(3, 0));
  }

private:
  void game_status_subscriber_callback_(
      const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        current_game_board_2D_[i][j] = msg->data[j + 3 * i];
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "Just received current game board's state:");
    print_game_board_2D();

    // get best move for the 0 player
    best_move_ = find_best_move();
    RCLCPP_INFO(this->get_logger(), "ORIGINAL best_move: {%d,%d}",
                best_move_.data[0], best_move_.data[1]);

    // publish best move WITH Current board vector
    for (size_t i = 0; i < msg->data.size(); i++) {
      best_move_.data.push_back(msg->data[i]);
    }
    std_msgs::msg::Int32MultiArray best = translate_to_flat_circle(best_move_);
    RCLCPP_INFO(this->get_logger(), "best_move: {%d,%d}", best.data[0],
                best.data[1]);
    best_move_publisher_->publish(best);
  }

  void print_game_board_2D() {
    for (size_t i = 0; i < current_game_board_2D_.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "%d | %d | %d",
                  current_game_board_2D_[0][i], current_game_board_2D_[1][i],
                  current_game_board_2D_[2][i]);
    }
    RCLCPP_INFO(this->get_logger(), "--------");
  }

  bool check_full_board() {
    bool is_full = true;
    for (size_t i = 0; i < current_game_board_2D_.size(); i++) {
      for (size_t j = 0; j < current_game_board_2D_[0].size(); j++) {
        if (current_game_board_2D_[i][j] == 0) {
          is_full = false;
          break;
        }
      }
    }
    return is_full;
  }

  int check_winner() {
    // check vertical winner
    for (auto &el : current_game_board_2D_) {
      if (el[0] == el[1] && el[1] == el[2] && el[2] != 0) {
        return el[0];
      }
    }
    // check horizontal winner
    for (size_t j = 0; j < current_game_board_2D_[0].size(); j++) {
      if (current_game_board_2D_[0][j] == current_game_board_2D_[1][j] &&
          current_game_board_2D_[1][j] == current_game_board_2D_[2][j] &&
          current_game_board_2D_[2][j] != 0) {
        return current_game_board_2D_[0][j];
      }
    }
    // check diagonal winner
    if (current_game_board_2D_[0][0] == current_game_board_2D_[1][1] &&
        current_game_board_2D_[1][1] == current_game_board_2D_[2][2] &&
        current_game_board_2D_[2][2] != 0) {
      return current_game_board_2D_[0][0];
    }
    if (current_game_board_2D_[0][2] == current_game_board_2D_[1][1] &&
        current_game_board_2D_[1][1] == current_game_board_2D_[2][0] &&
        current_game_board_2D_[2][0] != 0) {
      return current_game_board_2D_[0][2];
    }

    return 0;
  }

  std_msgs::msg::Int32MultiArray find_best_move() {
    int best_score = -100;

    // apply min_max algo on each free grid square
    for (size_t i = 0; i < current_game_board_2D_.size(); i++) {
      for (size_t j = 0; j < current_game_board_2D_[0].size(); j++) {
        if (current_game_board_2D_[i][j] == 0) {
          current_game_board_2D_[i][j] = 2;
          int score = min_max(0, false);
          current_game_board_2D_[i][j] = 0;
          if (score > best_score) {
            best_score = score;
            best_move_.data = {static_cast<int>(i), static_cast<int>(j)};
          }
        }
      }
    }
    return best_move_;
  }

  int min_max(int depth, bool is_maximizing) {
    int win_check = check_winner();
    if (win_check == 2) {
      RCLCPP_DEBUG(this->get_logger(), "O WIN");
      return 1;
    } else if (win_check == 1) {
      RCLCPP_DEBUG(this->get_logger(), "X WIN");
      return -1;
    } else if (check_full_board()) {
      RCLCPP_DEBUG(this->get_logger(), "NONE WIN");
      return 0;
    }

    // maximizing
    if (is_maximizing) {
      int best_score = -100;
      for (size_t i = 0; i < current_game_board_2D_.size(); i++) {
        for (size_t j = 0; j < current_game_board_2D_[0].size(); j++) {
          if (current_game_board_2D_[i][j] == 0) {
            current_game_board_2D_[i][j] = 2;
            int score = min_max(depth + 1, false);
            current_game_board_2D_[i][j] = 0;
            best_score = std::max(best_score, score);
          }
        }
      }
      return best_score;
    }

    // minimizing
    else {
      int best_score = 100;
      for (size_t i = 0; i < current_game_board_2D_.size(); i++) {
        for (size_t j = 0; j < current_game_board_2D_[0].size(); j++) {
          if (current_game_board_2D_[i][j] == 0) {
            current_game_board_2D_[i][j] = 1;
            int score = min_max(depth + 1, true);
            current_game_board_2D_[i][j] = 0;
            best_score = std::min(best_score, score);
          }
        }
      }
      return best_score;
    }
  }

  std_msgs::msg::Int32MultiArray
  translate_to_flat_circle(std_msgs::msg::Int32MultiArray best_move) {
    std_msgs::msg::Int32MultiArray ret;
    // add the position converted to a 1D array you know
    ret.data.push_back(best_move.data[0] * 3 + best_move.data[1]);
    // and say you want  to put a circle (the robot puts circle)
    ret.data.push_back(0);

    // then add current_board square infos
    for (size_t i = 2; i < best_move_.data.size(); i++) {
      ret.data.push_back(best_move_.data[i]);
    }

    return ret;
  }

  // Sub & Pub
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      game_status_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      best_move_publisher_;
  // Variables
  std::vector<std::vector<int>> current_game_board_2D_;
  std_msgs::msg::Int32MultiArray best_move_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinMaxTicTacToe>());
  rclcpp::shutdown();
  return 0;
}
