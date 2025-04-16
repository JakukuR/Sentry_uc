
#uint8 detect_flag 
#uint8 no_chance
#uint16 hp
#uint8 base_low_hp
#uint8 outpost_low_hp
#uint8 outpostbreak

uint8 heat_flag####

uint8 buff_activated####

uint16 remaining_time####

uint8 weak####

uint8 game_status

bool bullet_flag 

uint8 economy_status #0 1 2 2000

uint8 my_hp

uint8 base_hp #0 1-5 1/1000 5000

uint8 outpost_hp #0 1-5 1/300 1500

#manual mark
uint8 key 

float32 p_x

float32 p_y

#enemy
uint8 enemy_1_robot_hp #0 1-4 500

uint8 x #enemy_2_robot_hp #0 1-3 250

uint8 y #enemy_3_robot_hp #0 1-4 400

uint8 enemy_4_robot_hp #0 1-4 400

uint8 enemy_7_robot_hp #0 1-4 400

uint8 gain_point_status



#enemy_info
bool is_exist_enemy

uint8 enemy_num

float32 target_enemy_dis

uint16 target_enemy_hp

#target_enemy_position

#exchange blood bullet is or not