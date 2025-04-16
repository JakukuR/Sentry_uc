#include "rm_serial_driver/nav_to_pose_client.hpp"
namespace rm_serial_driver
{
NavToPoseClient::NavToPoseClient(const rclcpp::NodeOptions & options)
: Node("goalpose", options)
{
   RCLCPP_INFO(this->get_logger(), "Starting NavToPoseClient!");
    // 创建导航动作客户端
    action_client_ = rclcpp_action::create_client<NavigationAction>(
        this, "navigate_to_pose");
    nav_status = this->create_publisher<std_msgs::msg::Int8>("/nav_status", 10);
     // 等待导航动作服务器上线，等待时间为5秒
    while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "等待Action服务上线。");
    }

    goal_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose_", 10,
    std::bind(&NavToPoseClient::GoalPoseSubCallback, this, std::placeholders::_1));

    // 设置导航目标点
  //   auto goal_msg = NavigationAction::Goal();
  //   goal_msg.pose.header.frame_id = "map";  // 设置目标点的坐标系为地图坐标系
  //   goal_msg.pose.pose.position.x = pose_x;  
  //   goal_msg.pose.pose.position.y = pose_y; 
  // std::cout<<"aaaa"<<pose_x<<std::endl;
  //   auto send_goal_options =
  //       rclcpp_action::Client<NavigationAction>::SendGoalOptions();
  //   // 设置请求目标结果回调函数
  //   send_goal_options.goal_response_callback =
  //       [this](NavigationActionGoalHandle::SharedPtr goal_handle) {
  //         if (goal_handle) {
  //           RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
  //         }
  //       };
  //   // 设置移动过程反馈回调函数
  //   send_goal_options.feedback_callback =
  //       [this](
  //           NavigationActionGoalHandle::SharedPtr goal_handle,
  //           const std::shared_ptr<const NavigationAction::Feedback> feedback) {
  //         (void)goal_handle;  // 假装调用，避免 warning: unused
  //         RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f",
  //                     feedback->distance_remaining);
  //       };
  //   // 设置执行结果回调函数
  //   send_goal_options.result_callback =
  //       [this](const NavigationActionGoalHandle::WrappedResult& result) {
  //         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
  //         {
  //           status.data = 1;
  //           RCLCPP_INFO(this->get_logger(), "处理成功！");
  //         }
  //         else
  //         {
  //           status.data = 0;
  //           RCLCPP_INFO(this->get_logger(), "处理失败！");
  //         }
  //       };
  //   // 发送导航目标点
  //   action_client_->async_send_goal(goal_msg, send_goal_options);
};

void NavToPoseClient::GoalPoseSubCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{   
    pose_x = msg->pose.position.x;
    pose_y = msg->pose.position.y;  
    auto goal_msg = NavigationAction::Goal();
    goal_msg.pose.header.frame_id = "map";  // 设置目标点的坐标系为地图坐标系
    goal_msg.pose.pose.position.x = pose_x;  
    goal_msg.pose.pose.position.y = pose_y; 

    auto send_goal_options =
        rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    // 设置请求目标结果回调函数
    send_goal_options.goal_response_callback =
        [this](NavigationActionGoalHandle::SharedPtr goal_handle) {
          if (goal_handle) {
            RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            ssta.data = 1;
          }
          else ssta.data = 0;
        };
    // 设置移动过程反馈回调函数
    send_goal_options.feedback_callback =
        [this](
            NavigationActionGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<const NavigationAction::Feedback> feedback) {
          (void)goal_handle;  // 假装调用，避免 warning: unused
          // RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f",
          //             feedback->distance_remaining);
          // std::cout<<"aaaascssdsdsdsdsdsd      "<<feedback->distance_remaining<<std::endl;
          if (feedback->distance_remaining <= 0.5) 
          {
            status.data = 1;
            RCLCPP_INFO(this->get_logger(), "处理成功！");
            // std::cout<<"aaadddd"<<int(status.data)<<std::endl;
          }
          else
          // if (result.code == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) 
          {
            status.data = 0;
            RCLCPP_INFO(this->get_logger(), "处理失败！");
          }
        };
    // 设置执行结果回调函数
    send_goal_options.result_callback =
        [this](const NavigationActionGoalHandle::WrappedResult& result) {
          RCLCPP_INFO(this->get_logger(), "hahaha！");
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
          {
            status.data = 5;
            RCLCPP_INFO(this->get_logger(), "处理成功！");
          }
          else
          // if (result.code == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) 
          {
            status.data = 1;
            RCLCPP_INFO(this->get_logger(), "处理失败！");
          }
        };
    // 发送导航目标点
    action_client_->async_send_goal(goal_msg, send_goal_options);
    std::cout<<"aaadddd"<<int(status.data)<<std::endl;
    nav_status->publish(status);
    is_goal->publish(ssta);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::NavToPoseClient)