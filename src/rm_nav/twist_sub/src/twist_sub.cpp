
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.h>
class  twist_sub : public rclcpp::Node
{
public:
        twist_sub(std::string name) : Node(name){
            RCLCPP_INFO(this->get_logger(),"start node%s. ",name.c_str());
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",10,std::bind(&twist_sub::sentryNavCallback, this, std::placeholders::_1));
        }
public:
          double linear_x =0;
          double angular_z =0;
          double linear_y =0;
public:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        void sentryNavCallback(geometry_msgs::msg::Twist::SharedPtr msg){
             linear_x = msg->linear.x;
             angular_z = msg->angular.z;
             linear_y =msg->linear.y;
//  RCLCPP_INFO(this->get_logger(),"linear_x  %s , angular_z %s",msg->linear.x,msg->angular.z);
 std::cout<<"linear_x"<<linear_x<<std::endl;
  std::cout<<"angular_z"<<angular_z<<std::endl;
  std::cout<<"linear_y"<<"       "<<linear_y<<std::endl;
}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<twist_sub>("twist_sub");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

