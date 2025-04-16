// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
// #include "rm_serial_driver/pose.hpp"

namespace rm_serial_driver
{
int data_length = 18;
double x1,y1,x2,y2,z1,z2;
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  serial_port_pub_=this->create_publisher<auto_aim_interfaces::msg::SerialPort>("game_status", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
  nav_status_sub = this->create_subscription<std_msgs::msg::Int8>(
  "/nav_status",10,
  std::bind(&RMSerialDriver::NavStatusCallback, this, std::placeholders::_1));
  supply_status_sub = this->create_subscription<std_msgs::msg::Int8>(
  "/supply_status_",10,
  std::bind(&RMSerialDriver::SupplyStatusCallback, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "/cmd_vel",rclcpp::SensorDataQoS(),
  std::bind(&RMSerialDriver::sentryNavCallback, this, std::placeholders::_1));

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  
  // target_rune_sub_ = this->create_subscription<auto_aim_interfaces::msg::TargetRune>(
  //   "/tracker/target_rune", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendDataOfRune, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;

  data.reserve(sizeof(ReceivePacket));
  // data.reserve(sizeof(uint8_t) * 50);
  bzero(data.data(), sizeof(uint8_t) * 13);
  while (rclcpp::ok()) {
    try {
      bzero(data.data(), sizeof(uint8_t) * 13);
      serial_driver_->port()->receive(header);
      //std::cout<<"hhhhhhhhhhh"<<header[0]<<std::endl;

      // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "header.empty : %d", header.empty());

      if (header[0] == 0xAA) {
     
        data.resize(data_length-1);
        int len = serial_driver_->port()->receive(data);
               
      
        if(len < data_length-1 ){
          
          std::vector<uint8_t> data_(data_length - 1 - len);
          // auto first_time = this->now();

           serial_driver_->port()->receive(data_);

          //  auto final_time = this->now();
          // auto latency = (final_time - first_time).seconds() * 1000;
          // std::cout<<"   "<<latency<<std::endl;
          data.resize(len);
          data.insert(data.end(),data_.begin(),data_.end());    
        }
      
        data.insert(data.begin(), header[0]);



      // first_time = final_time;
        // if(data[17]!=0xAF)
        // {
        //   // std::cout<<"8  : "<<data[10]<<std::endl;/
        //   std::cout<<"qqqqqqqqqqqqqq"<<std::endl;
        //   continue;
        // }
        // else
        // {
        // std::cout << "1::::: " << header[0] << std::endl;
        // std::cout << "2::::: " << data[2]<< std::endl;

        // printf("1:::::                  %d\n", data[0]);
        // printf("2:::::                  %d\n", data[1]);
        // printf("3:::::                  %d\n", data[2]);
        // printf("4:::::                  %d\n", data[3]);
        // printf("5:::::                  %d\n", data[4]);
        // printf("6:::::                  %d\n", data[5]);
        // printf("7:::::                  %d\n", data[6]);
        // printf("8:::::                  %d\n", data[7]);
        // printf("9:::::                  %d\n", data[8]);
        // printf("0:::::                  %d\n", data[9]);


        // }
        ReceivePacket packet = fromVector(data);
        receive_yaw_ = packet.yaw_angle;
        receive_pitch_ = packet.pitch_angle;
        // std::cout<<"pitch"<<packet.pitch_angle<<std::endl;
        // std::cout << "receive yaw: " << packet.yaw_angle << "  receive pitch: " << packet.pitch_angle << std::endl;
        // std::cout<<std::to_string(data[6])<<std::endl;
        // bool crc_ok =
        //   crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        // if (crc_ok) {
        if (!initial_set_color_param_ || packet.detect_color != previous_receive_color_) {
          // setParam(rclcpp::Parameter("detect_color", packet.detect_color), "detect_color");
          // std::cout<<"detect_color:::::::::::::::::::"<<packet.detect_color<<std::endl;
          setParam(rclcpp::Parameter("detect_color", packet.detect_color), "detect_color");
          // int temp_color;
          // temp_color=get_parameter("detect_color").as_int();
          // std::cout<<"temp_color:::::::::"<<temp_color<<std::endl;
          // if(temp_color!=0||temp_color!=1){
          //   setParam(rclcpp::Parameter("detect_color", packet.detect_color), "detect_color");
          //   std::cout<<"temp_color:::::::::"<<temp_color<<std::endl;  std::cout<<packet.my_hp<<"    hppppp"<<std::endl;
          //  std::cout<<packet.base_hp<<"    base---hppppp"<<std::endl;
          // std::cout<<packet.outpost_hp<<"    op---hppppp"<<std::endl;
          // std::cout<<packet.bullet_flag<<"    ---bullet"<<std::endl;
          // }
          // previous_receive_color_ = packet.detect_color;
        }

        // RCLCPP_INFO(get_logger(), "receive aim mode: %d", packet.aim_mode);
        if (!initial_set_mode_param_ || packet.aim_mode != previous_aim_mode_) {
          setParam(rclcpp::Parameter("aim_mode", packet.aim_mode), "aim_mode");
          // setParam(rclcpp::Parameter("aim_mode", 3), "aim_mode");
          previous_aim_mode_ = packet.aim_mode;
        }

        //    不击打的数字
        // RCLCPP_INFO(get_logger(), "receive unrecognized_num: %d", packet.unrecognized_num);
        if (!initial_set_num_param_ || packet.unrecognized_num != previous_unrecognized_num_) {
          // setParam(rclcpp::Parameter("unrecognized_num", packet.unrecognized_num), "unrecognized_num");
          setParam(rclcpp::Parameter("unrecognized_num", packet.unrecognized_num), "unrecognized_num");
          previous_unrecognized_num_  = packet.unrecognized_num;
        }
        
        // if (packet.reset_tracker) {
        // resetTracker();
        // }
        shoot_speed = packet.bullet_speed;
        geometry_msgs::msg::TransformStamped t;
        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t.header.frame_id = "base_link"; //"livox_frame"
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q;
        q.setRPY(0, packet.pitch_angle / 180 * M_PI, packet.yaw_angle / 180 * M_PI);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
        receive_data = packet;
        auto_aim_interfaces::msg::SerialPort serial_nav;
        serial_nav.game_status = packet.game_status;
        serial_nav.my_hp = packet.my_hp;
        serial_nav.base_hp = packet.base_hp;
        serial_nav.outpost_hp = packet.outpost_hp;
        //serial_nav.outpostbreak = packet.outpostbreak;
        serial_nav.bullet_flag = packet.bullet_flag;
        serial_nav.economy_status = packet.economy_status;
        serial_nav.remaining_time = packet.remaining_time;
        serial_nav.buff_activated = packet.buff_activated;
        serial_nav.heat_flag = 0;
        serial_nav.key = packet.key;
        serial_nav.p_x = packet.p_x;
        serial_nav.p_y = packet.p_y;
        serial_nav.enemy_1_robot_hp = packet.enemy_1_robot_hp;
        serial_nav.x = packet.x;
        serial_nav.y = packet.y;
        serial_nav.enemy_4_robot_hp = packet.enemy_4_robot_hp;
        serial_nav.enemy_7_robot_hp = packet.enemy_7_robot_hp;
        serial_nav.gain_point_status = packet.gain_point_status;
        
        // serial_nav.game_status = 0;
        // std::cout<<"hp:gg  "<<packet.hp<<std::endl;
        // std::cout<<"p_x:     "<<packet.p_x<<std::endl;
        // std::cout<<"p_y:      "<<packet.p_y<<std::endl;
        // serial_nav.hp = 400;
        // serial_nav.key = 1;
        // serial_nav.heat_flag =0;
        // serial_nav.p_x = 4;
        // serial_nav.p_y = 2;

        //    std::cout<<packet.my_hp<<"    hppppp"<<std::endl;
        //    std::cout<<packet.base_hp<<"    base---hppppp"<<std::endl;
        //   std::cout<<packet.outpost_hp<<"    op---hppppp"<<std::endl;
        //   std::cout<<packet.bullet_flag<<"    ---bullet"<<std::endl;
        //  printf("game    %d\n",packet.game_status);
        // printf("eco    %d\n",packet.economy_status);
        // printf("e1   %d\n",packet.enemy_1_robot_hp);
        // printf("x   %d\n",packet.x);
        // printf("y   %d\n",packet.y);
        // printf("e4   %d\n",packet.enemy_4_robot_hp);                                                                                                                                `
        // printf("e7   %d\n",packet.enemy_7_robot_hp);
        // std::cout<<"ggggg      "<<packet.gain_point_status<<std::endl;
         //std::cout<<serial_nav.my_hp<<"hppppp"<<std::endl;

        serial_port_pub_->publish(serial_nav);
        // } else {
        //   RCLCPP_ERROR(get_logger(), "CRC error!");
        // }
      } else 
      // if(header[0] == 0xAC){
      //   // data.resize(6);
      //   // serial_driver_->port()->receive(data);

      //   // data.insert(data.begin(), header[0]);
      //   data.resize(6-1);
      //   int len = serial_driver_->port()->receive(data);
      //   if(len < 6-1 ){
          
      //     std::vector<uint8_t> data_(6 - 1 - len);
      //     // auto first_time = this->now();

      //      serial_driver_->port()->receive(data_);

      //     //  auto final_time = this->now();
      //     // auto latency = (final_time - first_time).seconds() * 1000;
      //     // std::cout<<"   "<<latency<<std::endl;
      //     data.resize(len);
      //     data.insert(data.end(),data_.begin(),data_.end());    
      //   }
      
      //   data.insert(data.begin(), header[0]);

      // //first_time = final_time;
      //   if(data[9]!=0xAF)
      //   {
      //     std::cout<<"qqqqqqqqqqqqqq"<<std::endl;
      //     continue;
      //   }
      //   ReceivePacket_Nav packet_nav = fromVector_nav(data);
      //   auto_aim_interfaces::msg::SerialPort serial_nav;
      //   serial_nav.game_status = packet_nav.game_status;
      //   serial_nav.hp = packet_nav.hp;
      //   serial_nav.remaining_time = packet_nav.remaing_time;
      //   serial_nav.buff_activated = packet_nav.buff_activated;
      //   serial_nav.base_low_hp = packet_nav.base_low_hp;
      //   serial_nav.enemy_base_low_hp = packet_nav.enemy_base_low_hp;
      //   serial_port_pub_=
      //   this->create_publisher<auto_aim_interfaces::msg::SerialPort>("game_status", 10);
      //   serial_port_pub_->publish(serial_nav);
      // }
      //  else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %04X", header[0]);
        // header.pop_back();
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}


// void RMSerialDriver::receiveData()
// {
//   std::vector<uint8_t> header(1);
//   std::vector<uint8_t> data;

//   data.reserve(20);
//   //data.reserve(sizeof(ReceivePacket)); //26个字节
//   // data.reserve(sizeof(uint8_t) * 50);
//   bzero(data.data(), sizeof(uint8_t) * 20);
//   while (rclcpp::ok()) {
//     try {
//       serial_driver_->port()->receive(header);

//       // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "header.empty : %d", header.empty());
//       ReceivePacket packet;
//       if (header[0] != 0xAA)
//       {
//         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %04X", header[0]);
//         std::vector<uint8_t> data_tmp;
//         int i=0;
//         for(i=0;i<10;++i)
//         {
//           if(header[i]==0xAA)
//           {
//             int j=0;
//             for(j=i;j<=(10+i);j++)
//             {
//                 data_tmp.push_back(header[j]);
//             }
//             break;
//           }
//         }
//         packet = fromVector(data_tmp);
//       }
//       // if (header[0] == 0xAA) {
//       else{
//         // data.resize(sizeof(ReceivePacket) - 1);
//         // serial_driver_->port()->receive(data);

//         // data.insert(data.begin(), header[0]);
//        // int i =1;
//         packet = fromVector(data);
//       }
        
//         receive_yaw_ = packet.yaw_angle;
//         std::cout<<"pitch"<<packet.pitch_angle<<std::endl;
//         // std::cout << "receive yaw: " << packet.yaw_angle << "  receive pitch: " << packet.pitch_angle << std::endl;
//         // std::cout<<std::to_string(data[6])<<std::endl;
//         // bool crc_ok =
//         //   crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
//         // if (crc_ok) {
//         if (!initial_set_color_param_ || packet.detect_color != previous_receive_color_) {
//           // setParam(rcllinear_xcpp::Parameter("detect_color", packet.detect_color), "detect_color");
//           setParam(rclcpp::Parameter("detect_color", 0), "detect_color");
//           previous_receive_color_ = packet.detect_color;
//         }
        
//         // RCLCPP_INFO(get_logger(), "receive aim mode: %d", packet.aim_mode);
//         if (!initial_set_mode_param_ || packet.aim_mode != previous_aim_mode_) {
//           setParam(rclcpp::Parameter("aim_mode", packet.aim_mode), "aim_mode");
//           // setParam(rclcpp::Parameter("aim_mode", 3), "aim_mode");
//           previous_aim_mode_ = packet.aim_mode;
//         }

//         //    不击打的数字
//         // RCLCPP_INFO(get_logger(), "receive unrecognized_num: %d", packet.unrecognized_num);
//         if (!initial_set_num_param_ || packet.unrecognized_num != previous_unrecognized_num_) {
//           // setParam(rclcpp::Parameter("unrecognized_num", packet.unrecognized_num), "unrecognized_num");
//           setParam(rclcpp::Parameter("unrecognized_num", packet.unrecognized_num), "unrecognized_num");
//           previous_unrecognized_num_  = packet.unrecognized_num;
//         }

        
//         // if (packet.reset_tracker) {
//         // resetTracker();
//         // }
//         shoot_speed = packet.bullet_speed;
//         geometry_msgs::msg::TransformStamped t;
//         timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
//         t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
//         t.header.frame_id = "odom";
//         t.child_frame_id = "gimbal_link";
//         tf2::Quaternion q;
//         q.setRPY(0, packet.pitch_angle / 180 * M_PI, packet.yaw_angle / 180 * M_PI);
//         t.transform.rotation = tf2::toMsg(q);
//         tf_broadcaster_->sendTransform(t);
//         receive_data = packet;
//         // } else {
//         //   RCLCPP_ERROR(get_logger(), "CRC error!");
//         // }
      
//       // else {
//       //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %04X", header[0]);
//       // }
//     } catch (const std::exception & ex) {
//       RCLCPP_ERROR_THROTTLE(
//         get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
//       reopenPort();
//     }
//   }
// }

// @brief
// @param twist_msg
void RMSerialDriver::sentryNavCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  try{
  SendPacket_Nav packet;
  packet.linear_x =1000*msg->linear.x;
  packet.linear_y = 1000*msg->linear.y;
  // packet.linear_x = 188.78;
  // packet.linear_y = 1;
  packet.angular_z=abs(float(nav_status));
  // std::cout<<"aaccacac"<<packet.angular_z<<std::endl;
  if(msg->angular.z>=0){
    flag=1;
  }
  else if (msg->angular.z<0)
  {
    flag=0;
  }
  
  if(msg->linear.y>=0 && msg->linear.x>=0){
    ly_flag=0;
  }
  else if (msg->linear.y<0 && msg->linear.x>=0)
  {
    ly_flag=1;
  }
  else if (msg->linear.y>=0 && msg->linear.x<=0)
  {
    ly_flag=0;
  }
  else if (msg->linear.y<0 && msg->linear.x<=0)
  {
    ly_flag=1;
  }
  

  if(msg->linear.x>=0){
    lx_flag=1;
  }
  else if (msg->linear.x<0)
  {
    lx_flag=0 ;
  }

  packet.flag=stop_flag<<3|flag<<2|lx_flag<<1|ly_flag;
  packet.supply_status=supply_status;
  // std::cout<<"aaccacac"<<packet.angular_z<<std::endl;
  std::vector<uint8_t>data = toVector_nav(packet);
  serial_driver_->port()->send(data);
  // std::cout<<"linear     "<<packet.linear_x<<std::endl;
  // std::cout<<"linear_y     "<<packet.linear_y<<std::endl;
  // std::cout<<"dtheta     linear_x"<<packet.angular_z<<std::endl;
  // std::cout<<"flag_y    "<<ly_flag<<std::endl;
  // std::cout<<"flag_x    "<<lx_flag<<std::endl;
  }
  catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data of nav_msg: %s", ex.what());
    reopenPort();
  }
  
}

void RMSerialDriver::NavStatusCallback(std_msgs::msg::Int8::SharedPtr msg)
{
  nav_status = msg->data;
}

void RMSerialDriver::SupplyStatusCallback(std_msgs::msg::Int8::SharedPtr msg)
{
  supply_status = msg->data;
}

/// @brief
/// @param target_msg
void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_uint8_map{
    {"", 0},  {"negative", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};
  std::string armor_id = msg->id;
  uint8_t id = id_uint8_map.at(msg->id);
  aiming_point_.header.frame_id = "base_link"; //"livox_frame"
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  try {
    SendPacket packet;

    struct tar_pos
    {
      double x;
      double y;
      double z;
      double yaw;
    };

    double yaw = msg->yaw;
    double r1 = msg->radius_1, r2 = msg->radius_2;
    double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
    double dz = msg->dz;
    double vx = msg->velocity.x, vy = msg->velocity.y;
    std::string type = msg->type;
    bool is_current_pair = true;
    size_t a_n = msg->armors_num;
    struct tar_pos tar_position[a_n];
    geometry_msgs::msg::Point p_w;
    auto_aim_interfaces::msg::DebugSend target_data;

    double armor_yaw[4];
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      if (tmp_yaw < 0) {
        tmp_yaw = 2 * M_PI + tmp_yaw;
      }
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        tar_position[i].z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        tar_position[i].z = za;
      }
      tar_position[i].x = xc - r * cos(tmp_yaw);
      tar_position[i].y = yc - r * sin(tmp_yaw);
      armor_yaw[i] = tmp_yaw;
      target_data.point.x = tar_position[i].x;
      target_data.point.y = tar_position[i].y;
      target_data.point.z = tar_position[i].z;
      target_data.yaw = tmp_yaw;
      target_data.id = i;
      target_data.angle = tmp_yaw * 180 / M_PI;
      this->debug_send_armors_msg_.data.emplace_back(target_data);
    }

    // Calculate center angle
    double center_yaw = 0;
    center_yaw = atan2(yc, xc);
    if (center_yaw > 2 * M_PI)
      center_yaw -= 2 * M_PI;
    else if (center_yaw < 0)
      center_yaw += 2 * M_PI;
    this->debug_send_armors_msg_.center_angle = center_yaw * 180 / M_PI;
    // RCLCPP_INFO_STREAM(get_logger(), "center_yaw: " + std::to_string(center_yaw* 180 / M_PI));
    //
    double bullet_speed = shoot_speed;
    double dis = sqrt(
      msg->position.x * msg->position.x + msg->position.y * msg->position.y +
      msg->position.z * msg->position.z);
    double time_delay = dis / bullet_speed + 0.1;
    int idx = 0;
    double pre_armor_yaw[4] = {0};
    for (size_t i = 0; i < a_n; i++) {
      pre_armor_yaw[i] = time_delay * msg->v_yaw + armor_yaw[i];
      if (pre_armor_yaw[i] > 4 * M_PI) {
        pre_armor_yaw[i] -= 4 * M_PI;
      }
      if (pre_armor_yaw[i] > 2 * M_PI) {
        pre_armor_yaw[i] -= 2 * M_PI;
      }
    }
    // std::cout<<"0:::"<<pre_armor_yaw[0]<<std::endl;
    // std::cout<<"1:::"<<pre_armor_yaw[1]<<std::endl;
    // std::cout<<"2:::"<<pre_armor_yaw[2]<<std::endl;
    // std::cout<<"3:::"<<pre_armor_yaw[3]<<std::endl;
    // std::cout<<"center_yaw"<<center_yaw<<std::endl;
    double yaw_diff_min = fabs(center_yaw - pre_armor_yaw[0]);
    if (yaw_diff_min > M_PI) {
      if (center_yaw >= 0 && center_yaw < M_PI)
        yaw_diff_min = 2 * M_PI - pre_armor_yaw[0] + center_yaw;
      else
        yaw_diff_min = 2 * M_PI - center_yaw + pre_armor_yaw[0];
    }
    // std::cout<<"yaw_diff1         :"<<yaw_diff_min<<std::endl;
    // std::cout<<"                          "<<std::endl;
    for (size_t i = 1; i < a_n; i++) {
      double temp_yaw_diff = fabs(center_yaw - pre_armor_yaw[i]);
      if (temp_yaw_diff > M_PI) {
        if (center_yaw >= 0 && center_yaw < M_PI)
          temp_yaw_diff = 2 * M_PI - pre_armor_yaw[i] + center_yaw;
        else
          temp_yaw_diff = 2 * M_PI - center_yaw + pre_armor_yaw[i];
      }
      // std::cout<<"yaw_diff        :"<<temp_yaw_diff<<std::endl;

      if (temp_yaw_diff < yaw_diff_min) {
        yaw_diff_min = temp_yaw_diff;
        idx = i;
      }
    }
    // tar_position[i].x = xc - r * cos(tmp_yaw);
    //   tar_position[i].y = yc - r * sin(tmp_yaw);

    p_w.x = xc - r * cos(pre_armor_yaw[idx]) + time_delay * vx;
    p_w.y = yc - r * sin(pre_armor_yaw[idx]) + time_delay * vy;
    p_w.z = tar_position[idx].z;
    Eigen::Vector3d xyz = {p_w.x, p_w.z, p_w.y};

    double pitch_offset = dynamicCalcPitchOffset(xyz);
    

    // if(type == "large")
    // {
    //   x1 = p_w.x - large_length * sin(pre_armor_yaw[idx]);
    //   y1 = p_w.y - large_length * cos(pre_armor_yaw[idx]);
    //   x2 = p_w.x + large_length * sin(pre_armor_yaw[idx]);
    //   y2 = p_w.y + large_length * cos(pre_armor_yaw[idx]);
    //   z1 = p_w.z - large_z;
    //   z2 = p_w.z + large_z;
    // }
    // else if(type == "small")
    // {
    //   x1 = p_w.x - small_length * sin(pre_armor_yaw[idx]);
    //   y1 = p_w.y - small_length * cos(pre_armor_yaw[idx]);
    //   x2 = p_w.x + small_length * sin(pre_armor_yaw[idx]);
    //   y2 = p_w.y + small_length * cos(pre_armor_yaw[idx]);
    //   z1 = p_w.z - small_z;
    //   z2 = p_w.z + small_z;
    // }
    // else{
    //   x1 = 0;
    //   y1 = 0;
    //   x2 = 0;
    //   y2 = 0;
    //   z1 = 0;
    //   z2 = 0;
    // }
    // double armor_yaw_err = fabs(atan2(y2,x2)/ M_PI * 180 - atan2(y1,x1)/ M_PI * 180);
    // if (armor_yaw_err > 360)
    //   armor_yaw_err -= 360;
    // if (armor_yaw_err > 180)
    //   armor_yaw_err -= 180;


    ///
    //调参 误差
    //下+ 右+？
    packet.yaw_angle = atan2(p_w.y, p_w.x) / M_PI * 180 + yaw_fix_;
    if (packet.yaw_angle > 360)
      packet.yaw_angle -= 360;
    else if (packet.yaw_angle < 0)
      packet.yaw_angle += 360;
    packet.pitch_angle =
      -atan2(p_w.z, sqrt(p_w.x * p_w.x + p_w.y * p_w.y)) / M_PI * 180 + 90 - pitch_offset+ pitch_fix_;
    ///
    
    
    packet.distance = sqrt(p_w.x * p_w.x + p_w.y * p_w.y + p_w.z * p_w.z);
    double a_distance = sqrt(p_w.x * p_w.x + p_w.y * p_w.y + p_w.z * p_w.z);
    double p_distance = sqrt(p_w.x * p_w.x + p_w.y * p_w.y );

    // double armor_pitch_error = fabs(atan2(z2,a_distance) / M_PI * 180 - atan2(z1,a_distance) / M_PI * 180);  /////////////////
    
    if (msg->tracking > 0)
      packet.detect_flag = id;
    else
      packet.detect_flag = 0;


///
    // detect_flag_pub_=
    //   this->create_publisher<auto_aim_interfaces::msg::DetectFlag>("/detect_flag", 10);
    //   if(packet.detect_flag>0)
    //   {
    //     send_detect_flag_msg.detect_flag=1;
    //     detect_flag_pub_->publish(send_detect_flag_msg);
    //   }
    //   else
    //   {
    //     send_detect_flag_msg.detect_flag=0;
    //     detect_flag_pub_->publish(send_detect_flag_msg);
    //   }

      // serial_port_pub_=
      // this->create_publisher<auto_aim_interfaces::msg::SerialPort>("game_status", 10);
      // if(packet.detect_flag>0)
      // {
      //   send_serial_port_msg.detect_flag=1;
      //   serial_port_pub_->publish(send_serial_port_msg);
      // }
      // else
      // {
      //   send_serial_port_msg.detect_flag=0;
      //   serial_port_pub_->publish(send_serial_port_msg);
      // }
      
      
///


    //packet.shoot_flag = 1;
    // std::cout<<"      aaa       "<<packet.detect_flag<<std::endl;
    double yaw_error = fabs(receive_yaw_-packet.yaw_angle);
    double pitch_error = fabs(receive_pitch_ - (packet.pitch_angle - 90));


    if (yaw_error > 180) {
      if (receive_yaw_ >= 0 && packet.yaw_angle >= 180)
        yaw_error = 360 - packet.yaw_angle + receive_yaw_;
      else
        yaw_error = 360 - receive_yaw_ + packet.yaw_angle;
    }

    packet.shoot_flag = 0;

    // if(packet.detect_flag !=0)
    // {
    //   if(yaw_error < armor_yaw_err && pitch_error < armor_pitch_error )
    //   {
    //      packet.shoot_flag = 1;
    //   }
    //   else{
    //     packet.shoot_flag = 0;
    //   }        
    // }
    std::cout<<"pitch_error : "<<pitch_error<<std::endl;
    std::cout<<"yaw_error            : "<<yaw_error<<std::endl;

    if(packet.detect_flag !=0){
      if(msg->armors_num == 2){
        if(yaw_error<(atan(0.17/a_distance))/M_PI*180 && pitch_error<(atan(0.14/p_distance))/M_PI*180){
          packet.shoot_flag = 1;
        }
          
      }else{
        if(id == 1){
          if(yaw_error<(atan(0.20/a_distance))/M_PI*180 && pitch_error<(atan(0.14/p_distance))/M_PI*180){
            packet.shoot_flag = 1;  
          }
            
        }else{
          if(yaw_error<(atan(0.135/a_distance))/M_PI*180 && pitch_error<(atan(0.14/p_distance))/M_PI*180){
            packet.shoot_flag = 1;
            // std::cout<<"angle"<<atan(0.15/a_distance)/M_PI*180<<std::endl;
          }

        }
      }
            // std::cout<<"angle"<<yaw_error<<std::endl;
    }
    if (!packet.detect_flag || (packet.pitch_angle - 90) < (-14)) {
      packet.yaw_angle = packet.pitch_angle = 0;
      packet.shoot_flag = 0;
      packet.detect_flag = 0;
    }
    //     std::cout<<"ayaw  ngle"<<packet.yaw_angle<<std::endl;

    // std::cout<<"pitch angle"<<packet.pitch_angle - 90<<std::endl;
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);

    // Publish aiming point
    if (marker_pub_->get_subscription_count() > 0) {
      aiming_point_.header.stamp = msg->header.stamp;
      aiming_point_.pose.position.x = p_w.x;
      aiming_point_.pose.position.y = p_w.y;
      aiming_point_.pose.position.z = p_w.z;
      marker_pub_->publish(aiming_point_);
    }
    //rm -r build
    //
    //source /opt/ros/humble/setup.bash 
    //colcon build --symlink-install
    this->send_armors_msg_.yaw_angle = packet.yaw_angle;
    this->send_armors_msg_.pitch_angle = packet.pitch_angle;
    this->send_armors_msg_.shoot_flag = packet.shoot_flag;
    this->send_armors_msg_.detect_flag = packet.detect_flag;
    this->send_armors_msg_.distance = packet.distance;

    // Publish send armors info
    send_armors_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::SendArmors>("/send_armors", 10);
    send_armors_pub_->publish(this->send_armors_msg_);
    // Publish debug info
    debug_send_armors_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugSendArmors>("/debug_send_armors", 10);
    debug_send_armors_pub_->publish(this->debug_send_armors_msg_);

 
    std_msgs::msg::Float64 latency;
    // std_msgs::msg::Float64 dec;
   // latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency.data = (float)packet.shoot_flag;
    // latency.data = (float)packet.detect_flag;
    latency_pub_->publish(latency);

    this->debug_send_armors_msg_.data.clear();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}


void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    yaw_fix_ = declare_parameter<double>("yaw_fix", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    pitch_fix_ = declare_parameter<double>("pitch_fix", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }
  try {
    large_length = declare_parameter<double>("large_length", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    large_z = declare_parameter<double>("large_z", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    small_length = declare_parameter<double>("small_length", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }
  
  try {
    small_z = declare_parameter<double>("small_z", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param, const std::string & name)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    if (name == "detect_color") {
      // RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
        {param}, [this, param](const ResultFuturePtr & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              // std::cout<<"color_error:::::::::::::::"<<get_parameter("detect_color").as_int()<<std::endl;
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          // RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
          initial_set_color_param_ = true;
        });
    } else if (name == "aim_mode") {
      RCLCPP_INFO(get_logger(), "Setting aim_mode to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
        {param}, [this, param](const ResultFuturePtr & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          RCLCPP_INFO(get_logger(), "Successfully set aim_mode to %ld!", param.as_int());
          initial_set_mode_param_ = true;
        });
    } else if (name == "unrecognized_num") {
      RCLCPP_INFO(get_logger(), "Setting unrecognized_num to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
        {param}, [this, param](const ResultFuturePtr & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          RCLCPP_INFO(get_logger(), "Successfully set unrecognized_num  to %ld!", param.as_int());
          initial_set_num_param_ = true;
        });
    }else {
      RCLCPP_ERROR(get_logger(), "Unknown parameter name: %s", name.c_str());
    }
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

double RMSerialDriver::dynamicCalcPitchOffset(Eigen::Vector3d & xyz)
{
  int max_iter = 10;
  float stop_error = 0.001;
  int R_K_iter = 50;
  // double bullet_speed_ = 28;
  // double bullet_speed = 16;            //TODO:弹速可变
  // const double k = 0.01903;
  const double k = 0.02903;  //25°C,1atm,小弹丸
  // const double k = 0.000556;                //25°C,1atm,大弹丸
  // const double k = 0.000530;                //25°C,1atm,发光大弹丸
  const double g = 9.788;

  auto bullet_speed = shoot_speed;
  //TODO:根据陀螺仪安装位置调整距离求解方式
  //降维，坐标系Y轴以垂直向上为正方向
  auto dist_vertical = xyz[1] ;
  auto vertical_tmp = dist_vertical;
  // auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
  // dist_horizonal = xyz[2];
  // dist_horizonal=6;
  // auto dist_vertical = xyz[2];
  auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
  // cout<<"dist.horizonal          "<<dist_horizonal<<endl;
  // std::cout<<"speed:::::::::::"<<bullet_speed<<std::endl;
  auto pitch = atan(dist_vertical / dist_horizonal) * 180 / M_PI;
  auto pitch_new = pitch;
  // auto pitch_offset = 0.0;

  //开始使用龙格库塔法求解弹道补偿
  for (int i = 0; i < max_iter; i++) {
    //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
    //初始化
    auto x = 0.0;
    auto y = 0.0;
    auto p = tan(pitch_new / 180 * M_PI);
    auto v = bullet_speed;
    auto u = v / sqrt(1 + pow(p, 2));
    auto delta_x = dist_horizonal / R_K_iter;
    for (int j = 0; j < R_K_iter; j++) {
      auto k1_u = -k * u * sqrt(1 + pow(p, 2));
      auto k1_p = -g / pow(u, 2);
      auto k1_u_sum = u + k1_u * (delta_x / 2);
      auto k1_p_sum = p + k1_p * (delta_x / 2);

      auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
      auto k2_p = -g / pow(k1_u_sum, 2);
      auto k2_u_sum = u + k2_u * (delta_x / 2);
      auto k2_p_sum = p + k2_p * (delta_x / 2);

      auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
      auto k3_p = -g / pow(k2_u_sum, 2);
      auto k3_u_sum = u + k3_u * (delta_x / 2);
      auto k3_p_sum = p + k3_p * (delta_x / 2);

      auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
      auto k4_p = -g / pow(k3_u_sum, 2);

      u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
      p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

      x += delta_x;
      y += p * delta_x;
    }
    //评估迭代结果,若小于迭代精度需求则停止迭代
    auto error = dist_vertical - y;
    if (abs(error) <= stop_error) {
      break;
    } else {
      vertical_tmp += error;
      // xyz_tmp[1] -= error;
      pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / M_PI;
    }
  }
  return pitch_new - pitch;
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
