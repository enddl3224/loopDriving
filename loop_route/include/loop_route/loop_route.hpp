#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace loop_route
{
class LoopRoute : public rclcpp::Node
{
public:
  explicit LoopRoute(const rclcpp::NodeOptions & node_options);

private:
  void callbackAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void callbackStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped msg);
  void callbackEndPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void callbackCheckPoint(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void callbackLoopLimit(const std_msgs::msg::UInt8 msg);
  
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_start_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_end_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_checkpoint_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_limit_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr checkpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_pub_;

  autoware_auto_system_msgs::msg::AutowareState autoware_state_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped end_pose_;
  geometry_msgs::msg::PoseStamped stop_by_point_;
  geometry_msgs::msg::PoseStamped checkpoint_;
  
  bool received_autoware_state_ = false;
  bool ready_to_publish_pose = false;
  bool goal_pose_published_ = false;
  bool engage_pose_published_ = false;
  bool received_first_goal_ = false;
  bool main_course_ = true;
  u_int8_t loop_limit_;
};
} //namespace loop_route