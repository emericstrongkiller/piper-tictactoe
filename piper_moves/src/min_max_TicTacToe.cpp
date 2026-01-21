#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

// use MinMax on every blank board place => retain the place that has the best
// score

// Convert this position back to a Int32MultiArray format for the game grid has
// 1 dimension

// output this position to the take_pic_node

using namespace std::placeholders;

class MinMaxTicTacToe : public rclcpp::Node {
public:
  MinMaxTicTacToe() : Node("minmax_tic_tac_toe") {
    game_status_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/take_pic_order_response", 10,
            std::bind(&MinMaxTicTacToe::game_status_subscriber_callback_, this,
                      _1));

    // variables init
    current_game_board_2D_.resize(3, std::vector<int>(3, 0));
    current_game_board_1D_ = std::make_shared<std_msgs::msg::Int32MultiArray>();
    current_game_board_1D_->data = {2, 1, 1, 2, 0, 0, 1, 0, 0};

    // create + print board matrix
    game_status_subscriber_callback_(current_game_board_1D_);

    // get best move for the 0 player
    best_move_ = find_best_move();
    RCLCPP_INFO(this->get_logger(), "best_move: {%d,%d}", best_move_[0],
                best_move_[1]);
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
    for (auto &el : current_game_board_1D_->data) {
      if (static_cast<int>(el) == 0) {
        is_full = false;
        break;
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

  std::vector<int> find_best_move() {
    int best_score = -100;
    std::vector<int> best_move = {0, 0};

    // apply min_max algo on each free grid square
    for (size_t i = 0; i < current_game_board_2D_.size(); i++) {
      for (size_t j = 0; j < current_game_board_2D_[0].size(); j++) {
        if (current_game_board_2D_[i][j] == 0) {
          current_game_board_2D_[i][j] = 2;
          int score = min_max(0, false);
          current_game_board_2D_[i][j] = 0;
          if (score > best_score) {
            best_score = score;
            best_move = {static_cast<int>(i), static_cast<int>(j)};
          }
        }
      }
    }
    return best_move;
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

  // Sub & Pub
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      game_status_subscriber_;
  // Variables
  std::vector<std::vector<int>> current_game_board_2D_;
  std_msgs::msg::Int32MultiArray::SharedPtr current_game_board_1D_;
  std::vector<int> best_move_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinMaxTicTacToe>());
  rclcpp::shutdown();
  return 0;
}