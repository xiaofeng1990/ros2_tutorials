#include <cstdio>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

class HelloWorldNode : public rclcpp::Node
{
public:
  HelloWorldNode() : Node("hello_world_node")
  {
    while (rclcpp::ok)
    {
      RCLCPP_INFO(this->get_logger(), "Hello world");
      sleep(1);
    }
    
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorldNode>());
  rclcpp::shutdown();
  
  printf("hello world learning_node package\n");
  return 0;
}
