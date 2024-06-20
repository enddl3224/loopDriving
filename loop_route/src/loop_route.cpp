#include "loop_route/loop_route.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace loop_route
{
using autoware_auto_system_msgs::msg::AutowareState;
using autoware_adapi_v1_msgs::msg::OperationModeState;

LoopRoute::LoopRoute(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("loop_route", node_options)
{
  // subscribe
  sub_start_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_start_pose", rclcpp::QoS{1}, std::bind(&LoopRoute::callbackStartPose, this, std::placeholders::_1));
  sub_end_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input_end_pose", rclcpp::QoS{1}, std::bind(&LoopRoute::callbackEndPose, this, std::placeholders::_1));
  sub_state_ = create_subscription<AutowareState>(
    "input_autoware_state", rclcpp::QoS{1}, std::bind(&LoopRoute::callbackAutowareState, this, std::placeholders::_1));

  rclcpp::QoS qos_settings(1);
  qos_settings.durability(rclcpp::DurabilityPolicy::TransientLocal);
  // publish
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("output_goal", rclcpp::QoS{1});
  state_pub_ = create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("output_state", qos_settings);
}

// get autoware state
void LoopRoute::callbackAutowareState(const AutowareState::ConstSharedPtr msg)
{
  autoware_state_ = *msg;
  received_autoware_state_ = true;

  // 주행을 했는지 확인, 주행을 했다면 true
  if (autoware_state_.state == AutowareState::DRIVING){
    ready_to_publish_pose = true;
    goal_pose_published_ = false;
  }

  // MainCource 주행완료 후 goal pose를 기다리는 상태라면 goal_pose publish 하기
  if (ready_to_publish_pose && !goal_pose_published_
      && autoware_state_.state == AutowareState::WAITING_FOR_ROUTE){
    RCLCPP_INFO(this->get_logger(),"main_course_: %d",main_course_);

    geometry_msgs::msg::PoseStamped goal_pose;
    // main course를 달리고 있을 경우, 다음 goal_pose는 initial_pose 위치로 가야한다.
    if (main_course_){
      goal_pose.header = start_pose_.header;
      goal_pose.pose = start_pose_.pose;
      main_course_ = false;
    } else {
      goal_pose.header = end_pose_.header;
      goal_pose.pose = end_pose_.pose;
      main_course_ = true;
    }

    pose_pub_->publish(goal_pose);
    goal_pose_published_ = true;
  }
}

// get initial pose(=start pose)
void LoopRoute::callbackStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  if(!received_autoware_state_){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Not Received input_autoware_state");
    return;
  }

  start_pose_.header = msg.header;
  start_pose_.pose = msg.pose.pose;
  // RCLCPP_INFO(this->get_logger(),"start_pose_.pose.position.x: %f",start_pose_.pose.position.x);
}


// get goal pose(=end pose)
void LoopRoute::callbackEndPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if(!received_autoware_state_){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Not Received input_autoware_state");
    return;
  }

  // 첫 goal_pose만 end_pose에 들어 가야함
  if(received_first_goal_){
    end_pose_.header = msg->header;
    end_pose_.pose = msg->pose;
    received_first_goal_ = false;
    // RCLCPP_INFO(this->get_logger(),"end_pose_.pose.position.x: %f",end_pose_.pose.position.x);
  }
}

} //namespace loop_route