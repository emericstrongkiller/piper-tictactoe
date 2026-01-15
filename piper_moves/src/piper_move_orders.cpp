#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <memory>

class PiperMoveOrders : public rclcpp::Node {
public:
  PiperMoveOrders() : Node("piper_move_orders") {
    order_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/piper_move_order", 1);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr order_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PiperMoveOrders>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}