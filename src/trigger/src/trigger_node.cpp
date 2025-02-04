#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class TriggerNode : public rclcpp::Node
{
public:
  TriggerNode() : Node("trigger_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("trigger", 10);
    RCLCPP_INFO(this->get_logger(), "Trigger node started. Press ENTER to trigger...");
    timer_ = this->create_wall_timer(100ms, std::bind(&TriggerNode::checkForTrigger, this));
  }

private:
  void checkForTrigger()
  {
    // Check if there's a newline waiting in standard input.
    if (std::cin.peek() == '\n') {
      // Clear the newline from the stream.
      std::cin.ignore();
      std_msgs::msg::Empty msg;
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Trigger published.");
    }
  }

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TriggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
