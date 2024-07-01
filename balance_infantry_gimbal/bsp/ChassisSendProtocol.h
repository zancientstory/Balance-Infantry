#ifndef CHASSIS_SEND_PROTOCAL_H
#define CHASSIS_SEND_PROTOCAL_H

#include "string.h"
#include "struct_typedef.h"
#include "RefereeBehaviour.h"


typedef __PACKED_STRUCT
{
	uint8_t robot_id;
	uint8_t power_output;
	uint16_t remain_HP;
	uint16_t max_HP;
} robot_information_t;

typedef __PACKED_STRUCT
{
	uint16_t hero_remain_HP;
	uint16_t infantry3_remain_HP;
	uint16_t infantry4_remain_HP;
	uint16_t infantry5_remain_HP;
} enemy_information_t;

typedef __PACKED_STRUCT
{
	uint8_t game_status;
	uint16_t end_time;
} send_game_status_t;


extern uint8_t *send_power_heat_data(void);
extern uint8_t *send_bullet_speed(void);
extern uint8_t *send_bullet_limit(void);
extern uint8_t *send_power_limit(void);
extern uint8_t *send_robot_information(void);
extern uint8_t *send_enemy_information(void);
extern uint8_t *send_game_status(void);
#endif