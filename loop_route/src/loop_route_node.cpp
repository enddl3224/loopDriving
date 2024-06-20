#include "rclcpp/rclcpp.hpp"
#include "loop_route/loop_route.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<loop_route::LoopRoute>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}