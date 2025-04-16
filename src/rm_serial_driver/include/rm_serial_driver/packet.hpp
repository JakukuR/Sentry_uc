// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <iostream>
#include <bitset>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0xAA;  //0
  float yaw_angle;        //123
  float pitch_angle;      //45
  int aim_mode;           //6
  int detect_color;       //6
  double bullet_speed;  
  int unrecognized_num;   //7

  int game_status; 
  int my_hp; 
  int base_hp;   
  int outpost_hp;
  
  int key;
  float p_x;
  float p_y;

  int economy_status;
  int bullet_flag;
  int remaining_time;
  int buff_activated;
  //int heat_flag;

  int enemy_1_robot_hp; 
 // int enemy_2_robot_hp;
  //int enemy_3_robot_hp;
  int x;
  int y;
  int enemy_4_robot_hp;
  int enemy_7_robot_hp;
  int gain_point_status;

  uint8_t tail;
} __attribute__((packed));

// struct ReceivePacket_Nav
// {
//   uint8_t header = 0xAC;  //0
//   int game_status;        //1
//   int hp;      //2
//   int remaing_time;           //34
//   int buff_activated;       //5
//   int base_low_hp;       //5
//   int enemy_base_low_hp;   //6
//   uint8_t tail;
// } __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xAA;
  float yaw_angle;
  float pitch_angle;
  float distance;
  int shoot_flag = 0;
  int detect_flag;
} __attribute__((packed));

struct SendPacket_Nav{
  uint8_t header =0xAC;
  float linear_x;
  float linear_y;
  // int l_xflag;
  int supply_status;
  int flag;
  float angular_z;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & r_data)
{
  ReceivePacket packet;
  // printf("1:::::                  %d\n", r_data[0]);
  packet.yaw_angle = r_data[1] * 100 + r_data[2] + (float)r_data[3] / 100;
  if (r_data[4] - 90 < 0) {
    packet.pitch_angle = int(r_data[4] - 90) + 1.0 - (float)(r_data[5] / 100.0);
  } else {
    packet.pitch_angle = int(r_data[4] - 90) + (float)(r_data[5] / 100.0);
  }
  packet.aim_mode = 0;
  // packet.my_hp = r_data[6];
  packet.detect_color = int(r_data[6] % 10);
  //        rune_detector.enemy_color = RuneDetector::Color((r_data[6]%10+1)%2);
  packet.bullet_speed = double(r_data[7]) / 10;

  if (r_data[7] == 0) {
    packet.bullet_speed = 25;
  }
  if (packet.bullet_speed < 20) {
    packet.bullet_speed = 25;
  }
      //  std::cout<< " packet.bullet_speed   "<<packet.bullet_speed<<std::endl;

  packet.aim_mode = (packet.aim_mode == 1 || packet.aim_mode >= 4) ? 0 : packet.aim_mode;
  // printf("unrecognized_num---%d\n",r_data[8]);
  //不击打的数字
  // uint8_t num[5]={0};
  // num[0]=(r_data[8]>>4) & 1;  
  // num[1]=(r_data[8]>>3) & 1;  
  // num[2]=(r_data[8]>>2) & 1;  
  // num[3]=(r_data[8]>>1) & 1;  
  // num[4]=r_data[8] & 1;

  // packet.unrecognized_num=num[0]*70000+num[1]*5000+num[2]*400+num[3]*30+num[4]*1;              

  // packet_nav.remaing_time =r_data[3] * 100 + r_data[4];
  
  // packet.outpost_low_hp = r_data[9] & 1;
  // packet.base_low_hp = (r_data[9] & (1<<2))>>2;
  // packet.outpostbreak= (r_data[9] & (1<<3))>>3;   
  // packet.game_status = (r_data[9] & (1<<4))>>4; 
  // packet.economy_status = (r_data[9] & (1<<5))>>5;
  // packet.detect_color = (r_data[9] & (1<<6))>>6; 

  // std::cout<<"color : "<<packet.detect_color<<std::endl;
  // packet.bullet_flag = (r_data[9] & (1<<7))>>7; 
  // ////
  // packet.key = (r_data[8] & (1<<5))>>5;
  
  // packet.p_x = (r_data[10] | (r_data[11]&240)<<4)*1.0/100;
  // packet.p_y = ((r_data[11]&15)<<8 | r_data[12])*1.0/100; 
//  std::cout<<"hp"<<packet.hp<<std::endl;
  // std::cout<<"baselow"<<packet.base_low_hp<<std::endl;
////////////////////////////////////////////////////////////////////////
  packet.game_status = (r_data[9] & 0b00000111);
  packet.economy_status = (r_data[9] & 0b00011000) >>3;
  packet.my_hp = (r_data[9] & 0b11100000) >>5;
  packet.bullet_flag =r_data[10] & 1;
  packet.base_hp = (r_data[10] & 0b00001110) >>1;
  packet.outpost_hp = (r_data[10] & 0b01110000) >>4;
  packet.enemy_1_robot_hp = (r_data[11] & 0b00000111);
  //packet.enemy_2_robot_hp = (r_data[11] & 0b00011000) >>3;
  packet.x = (r_data[11] & 0b00011000) >>3;
  packet.y = (r_data[11] & 0b11100000) >>5;
  //packet.enemy_3_robot_hp = (r_data[11] & 0b11100000) >>5;
  packet.enemy_4_robot_hp = (r_data[12] & 0b00000111) ;
  packet.enemy_7_robot_hp = (r_data[12] & 0b00111000) >>3;
  packet.gain_point_status = (r_data[12] & 0b11000000) >>6;
  
  return packet;
}

// inline ReceivePacket_Nav fromVector_nav(const std::vector<uint8_t> & r_data)
// {
//   ReceivePacket_Nav packet_nav;

//   packet_nav.game_status = r_data[11];
//   packet_nav.hp = r_data[10]*3;
//   // packet_nav.remaing_time =r_data[3] * 100 + r_data[4];
  
//   packet_nav.buff_activated = r_data[9] & 3;
//   packet_nav.base_low_hp = (r_data[9] & (1<<2))>>2;
//   packet_nav.enemy_base_low_hp= (r_data[9] & (1<<3))>>3;    
//   return packet_nav;
// }

inline std::vector<uint8_t> toVector(const SendPacket & send_data)
{
  std::vector<uint8_t> packet(sizeof(uint8_t) * 9);

  packet[0] = send_data.header;
  packet[1] = (unsigned char)((int)send_data.yaw_angle / 100);    // integer part
  packet[2] = (unsigned char)((int)(send_data.yaw_angle) % 100);  // fractional part
  packet[3] = (unsigned char)((int)(send_data.yaw_angle * 100) % 100);

  packet[4] = (unsigned char)((int)send_data.pitch_angle % 256);
  packet[5] = (unsigned char)(((int)(send_data.pitch_angle * 100) % 100));
  packet[6] = (unsigned char)(int(send_data.distance / 100));  // High eight
  packet[7] =
    (unsigned char)(send_data.detect_flag + 100 * send_data.shoot_flag);  // detection marker
  // std::cout<<"shoot_flag : "<<send_data.shoot_flag<<std::endl;
  return packet;
}

inline std::vector<uint8_t> toVector_nav(const SendPacket_Nav & send_data)
{
  std::vector<uint8_t> packet(sizeof(uint8_t) * 9);

  packet[0] = send_data.header;
  packet[1] = (unsigned char)((int)abs(send_data.linear_x)/ 100);    // integer part
  packet[2] = (unsigned char)((int)(abs(send_data.linear_x)) % 100); // fractional part
  packet[3] = (unsigned char)((int)(send_data.supply_status));
  packet[4] = (unsigned char)((int)abs(send_data.angular_z) / 100);
  packet[5] = (unsigned char)((int)abs(send_data.angular_z) % 100);
  packet[6] = (unsigned char)((int)send_data.flag);
  packet[7] = (unsigned char)((int)abs(send_data.linear_y) / 100);
  packet[8] = (unsigned char)((int)(abs(send_data.linear_y)) % 100);
std::cout<<"linear_x     "<<send_data.linear_x<<std::endl;
std::cout<<"linear_y     "<<send_data.linear_y<<std::endl;
return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
