#ifndef REFEREE_BEHAVIOUER_H
#define REFEREE_BEHAVIOUER_H

#include "main.h"
#include "ChassisSendProtocol.h"
#include "bsp_referee.h"
#include "gpio.h"
#define SEND_BUS hcan1


#define SIZE 517

typedef struct
{
    uint8_t arr[SIZE];
    uint32_t front;
    int32_t rear;
    uint32_t counter;

}Queue_t;

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __PACKED_STRUCT //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef __PACKED_STRUCT //0002
{
    uint8_t winner;
} ext_game_result_t;

typedef __PACKED_STRUCT	//0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;

	uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;;

	uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef __PACKED_STRUCT //0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __PACKED_STRUCT //0x0102
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __PACKED_STRUCT //0x0104
{
    uint8_t level;
    uint8_t foul_robot_id;
    uint8_t count;
} ext_referee_warning_t;

typedef __PACKED_STRUCT //0x0105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_remaining_time_t;

typedef __PACKED_STRUCT//0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __PACKED_STRUCT //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __PACKED_STRUCT //0x0203
{
    float x;
    float y;
    float yaw;
} ext_game_robot_pos_t;

typedef __PACKED_STRUCT //0x0204
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} ext_buff_musk_t;

typedef __PACKED_STRUCT //0x0205
{
    uint8_t airforce_status;
    uint8_t time_remain;
} aerial_robot_energy_t;

typedef __PACKED_STRUCT //0x0206
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __PACKED_STRUCT //0x0207
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

typedef __PACKED_STRUCT //0x0208
{
uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __PACKED_STRUCT //0x0209
{
	uint32_t rfid_status;
} ext_rfid_status_t;

typedef __PACKED_STRUCT //0x020A
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

typedef __PACKED_STRUCT //0x0301
{
    uint16_t data_cmd_id;
	uint16_t sender_ID;
    uint16_t receiver_ID;
    uint8_t user_data[113];
} ext_student_interactive_data_t;

typedef __PACKED_STRUCT //0x0302
{
    uint8_t data[30];
} custom_data_t;

typedef __PACKED_STRUCT //0x304
{
    __PACKED_STRUCT
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __PACKED_STRUCT
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __PACKED_STRUCT
        {
                uint16_t v;
        } key;
		uint16_t reserved;
		uint16_t KeyFormerChannal;
		uint16_t KeyJumpChannal;
		uint16_t KeyUsed;
}remote_control_t;

typedef __PACKED_STRUCT // 0x0308
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
}
custom_info_t;

typedef __PACKED_STRUCT
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __PACKED_STRUCT
{
    uint8_t data[32];
} ext_download_stream_data_t;

extern frame_header_struct_t referee_receive_header;
extern frame_header_struct_t referee_send_header;

extern remote_control_t TuRc;

extern ext_game_state_t game_state;
extern ext_game_result_t game_result;
extern ext_game_robot_HP_t game_robot_HP_t;

extern ext_event_data_t field_event;
extern ext_supply_projectile_action_t supply_projectile_action_t;
extern ext_referee_warning_t referee_warning_t;


extern ext_game_robot_status_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_pos_t game_robot_pos_t;
extern ext_buff_musk_t buff_musk_t;
extern aerial_robot_energy_t robot_energy_t;
extern ext_robot_hurt_t robot_hurt_t;
extern ext_shoot_data_t shoot_data_t;
extern ext_bullet_remaining_t bullet_remaining_t;
extern ext_rfid_status_t rfid_status_t;
extern ext_student_interactive_data_t student_interactive_data_t;

extern ext_dart_client_cmd_t dart_client_cmd_t;
extern ext_dart_remaining_time_t dart_remaining_time_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern bool_t get_rfid_status(void);
extern void get_level(uint8_t *level);
extern void get_chassis_power_limit(uint16_t *power_limit);
extern void get_chassis_power_and_buffer(fp32 *power, uint16_t *power_limit, uint16_t *buffer);
extern uint8_t get_robot_id(void);
extern void get_shoot_heat1_limit_and_heat2(uint16_t *heat1_limit, uint16_t *heat1, uint16_t *head1_rate);
extern void get_shoot_heat2_limit_and_heat2(uint16_t *heat2_limit, uint16_t *heat2, uint16_t *head2_rate);

extern remote_control_t *get_remotetu_control_point(void);

extern Queue_t SendBuffer[2];
void QueueInit(Queue_t *q);
uint8_t IsFull(Queue_t *q);
void EnQueue(Queue_t *q, uint8_t *val,uint8_t lenth);
uint8_t IsEmpty(Queue_t *q);
int Pop(Queue_t *buffer1,Queue_t *buffer2,uint8_t data[11]);
#endif
