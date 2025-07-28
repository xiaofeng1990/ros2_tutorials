#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

rclcpp::Node::SharedPtr g_node = nullptr;
void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AddTwoInts::Request> request,
    std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header; // unused
  RCLCPP_INFO(g_node->get_logger(), "Incoming request\na: %d b: %d", request->a, request->b);
  response->sum = request->a + request->b;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("minimal_service");
  auto service = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
  RCLCPP_INFO(g_node->get_logger(), "Ready to add two ints.");
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}