#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp" // CHANGE

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{

public:
  PublisherNode() : Node("topic_publisher")
  {
    this->publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("chatter", 10);
    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&PublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();
    message.num = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);
    this->publisher_->publish(message);
  }
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}