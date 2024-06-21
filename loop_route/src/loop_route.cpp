#include "loop_route/loop_route.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace loop_route
{
using autoware_auto_system_msgs::msg::AutowareState;

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

  // publish
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("output_goal", rclcpp::QoS{1});
  engage_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("output_engage", rclcpp::QoS{1});
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
    engage_pose_published_ = false;
  }

  // MainCource 주행완료 후 goal pose를 기다리는 상태라면 goal_pose publish 하기
  if (ready_to_publish_pose && !goal_pose_published_
      && autoware_state_.state == AutowareState::WAITING_FOR_ROUTE){
    RCLCPP_INFO(this->get_logger(),"main_course_: %d",main_course_);

    geometry_msgs::msg::PoseStamped goal_pose;
    // main course를 달리고 있을 경우, 다음 goal_pose는 initial_pose 위치로 가야한다.
    // 주의) main course를 주행 완료하고 다시 출발지로 돌아가는 과정에서 initial pose를 변경하게 되면 (=[init by GNSS] button 클릭시)
    //      변경된 initial_pose(start_pose_)가 goal_pose로 들어가지 않기 때문에 원하는 loop가 생성되지 않는다
    //      (loop 한바퀴는 돌고 나서 initial pose를 변경하거나 main course 주행 시작 전에 initial_pose를 생성해야함)
    //      ** initial_pose와 goal_pose가 변경될 경우는 주로 주행을 시작하기 전에 일어나기 때문에 아직은 예외처리 하지 않음..
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

  // publish engage (AUTO button 눌러주기)
  if(goal_pose_published_ && !engage_pose_published_ 
      && autoware_state_.state == AutowareState::WAITING_FOR_ENGAGE){
    autoware_auto_vehicle_msgs::msg::Engage engage_for_auto;
    engage_for_auto.stamp = this->get_clock()->now();
    engage_for_auto.engage = true;
    engage_pub_->publish(engage_for_auto);
    engage_pose_published_ = true;
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
}


// get goal pose(=end pose)
void LoopRoute::callbackEndPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"goal_pose_published_: %d",goal_pose_published_);
  if(!received_autoware_state_){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Not Received input_autoware_state");
    return;
  }

  /*
    ===================================================================
    goal_pose를 다시 눌러 수정하고 싶을 경우,
    msg에 들어오는 값에서 이전의 goal_pose와 수정할 새로운 goal_pose를 구분 해야함
    -> 데이터가 들어온 time을 기준으로 구분
    ===================================================================
  */
  // 현재 시간 가져오기
  auto now = this->get_clock()->now();
  // msg의 타임스탬프와 현재 시간의 차이 계산
  rclcpp::Duration time_difference = now - msg->header.stamp;
  double time_difference_seconds = time_difference.seconds();
  // end_pose_의 타임스탬프와 현재 시간의 차이 계산
  rclcpp::Duration end_difference = now - end_pose_.header.stamp;
  double end_difference_seconds = end_difference.seconds();
  // initial_pose가 goal pose로 잡힐 경우를 제외하고 새로운 goal_pose를 생성했을 경우
  // 새로운 goal_pose를 end_pose에 넣어야 함 (= goal_pose를 아직 받지 않은 것과 동일함)
  if((start_pose_.header.stamp != msg->header.stamp)
      && (end_difference_seconds>time_difference_seconds)){
    RCLCPP_INFO(this->get_logger(), "!!change a new goal pose!!");
    received_first_goal_ = false;
  }

  // /planning/mission_planning/goal topic이 매번 바뀌기 때문에 원하는 goal_pose만 골라서 end_pose에 들어 가야함
  if(!received_first_goal_){
    end_pose_.header = msg->header;
    end_pose_.pose = msg->pose;
    received_first_goal_ = true;
  }
}

} //namespace loop_route