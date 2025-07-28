#include <memory>

#include "rclcpp/rclcpp.hpp"               // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"         // 字符串消息类型
#include "tutorial_interfaces/msg/num.hpp" // CHANGE

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
      : Node("topic_sub")
  {
    // 创建一个订阅者，订阅名为“topic”的话题，消息类型为std_msgs::msg::String，队列长度为10
    // 回调函数为topic_callback
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>("chatter", 10, std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.num);
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}