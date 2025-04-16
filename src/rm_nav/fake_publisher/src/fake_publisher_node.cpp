#include "rclcpp/rclcpp.hpp"
#include "decision_msgs/msg/serial_port.hpp"
#include "auto_aim_interfaces/msg/serial_port.hpp"
#include "std_msgs/msg/int8.hpp"
// class fake_message : public rclcpp::Node
// {
//     public:
//         fake_message():Node("fake_publisher_node")
//         {
//             decision_msgs::msg::SerialPort message;
//             // RCLCPP_INFO(this->get_logger(),"测试开始");
//             rclcpp::Publisher<decision_msgs::msg::SerialPort>::SharedPtr message_publisher;
//             message_publisher=this->create_publisher<decision_msgs::msg::SerialPort>("game_status",20);
//             message.game_status=1;
//             message.remaining_time=20;
//             message.base_low_hp=0;
//             message.buff_activated=0;
//             message.enemy_base_low_hp=0;
//             message.hp=200;
//             message.self_color=0;
//             message_publisher->publish(message);
//             // std::cout<<"AAaaa"<<std::endl;
//             //  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&fake_message::timer_callback, this));
//         }
//     void timer_callback()
//     {
//         // 创建消息
//         decision_msgs::msg::SerialPort message;
//             message.game_status=1;
//             message.remaining_time=20;
//             message.base_low_hp=0;
//             message.buff_activated=0;
//             message.enemy_base_low_hp=0;
//             message.hp=200;
//             message.self_color=0;
//         // 日志打印
//         // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//         // 发布消息
//         message_publisher->publish(message);
//     }
//     // 声名定时器指针
//     rclcpp::TimerBase::SharedPtr timer_;
//      rclcpp::Publisher<decision_msgs::msg::SerialPort>::SharedPtr message_publisher;
// };


// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     /*创建对应节点的共享指针对象*/
//     // std::cout<<"AAaaa"<<std::endl;
//     auto node = std::make_shared<fake_message>();
//     /* 运行节点，并检测退出信号*/
// //       while(rclcpp::ok())
// //   {
//     rclcpp::spin(node);
// //   }
//     rclcpp::shutdown();
//     return 0;
// }

class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01() : Node("topic_publisher_01")
    {
        // RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者
        command_publisher_ = this->create_publisher<auto_aim_interfaces::msg::SerialPort>("game_status", 10);
        command_publisher_b = this->create_publisher<std_msgs::msg::Int8>("/nav_status", 10);
           timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
    }
private:
    // 声明节点
     void timer_callback()
    {
        // 创建消息
         std_msgs::msg::Int8 a;
         auto_aim_interfaces::msg::SerialPort message;
         a.data =0;

        message.game_status=1;
        message.remaining_time=210;
        message.base_hp=0;
        message.buff_activated=0;

        message.my_hp=1;
        message.key=0;
        message.p_x = 0;
        message.p_y = 1;
        message.heat_flag =0;
        message.outpost_hp =1;
        message.economy_status =1;
        message.bullet_flag = 0;
        message.outpost_hp =0;
            // message.self_color=0;
        // 日志打印
        // RCLCPP_INFO(this->get_logger(),message.remaining_time);
        std::cout<<message.base_hp<<std::endl;
        // 发布消息
        command_publisher_->publish(message);
        command_publisher_b->publish(a);
    }
    rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<auto_aim_interfaces::msg::SerialPort>::SharedPtr command_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr command_publisher_b;
       // rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr command_publisher_c;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>();
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
