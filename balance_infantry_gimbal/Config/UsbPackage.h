#ifndef UAB_PACKAGE_H
#define UAB_PACKAGE_H

#include "struct_typedef.h"
#include "main.h"

//#define REFEREE_DATA_ID 0x1
//#define AIMBOT_REFEREE_DATA_ID 0x1
//#define GIMBAL_DATA_ID 0x3
//#define NUC_CONTROL_DATA_ID 0x4
#define GIMBAL_IMU_0_ID 0x1
//#define GIMBAL_IMU_1_ID 0x6
#define AIMBOT_DATA_0_ID 0x2
//#define AIMBOT_DATA_1_ID 0x8

// 49字节
typedef __PACKED_STRUCT
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // game state
    uint8_t game_type_progress;
    uint16_t game_stage_remain_time;
    // enemy information and outpose base HP
    uint16_t hero_remain_HP;
    uint16_t engineer_remain_HP;
    uint16_t infantry3_remain_HP;
    uint16_t infantry4_remain_HP;
    uint16_t infantry5_remain_HP;
    uint16_t sentry_remain_HP;
    uint16_t red_outpose_HP;
    uint16_t blue_outpose_HP;
    uint16_t red_base_HP;
    uint16_t blue_base_HP;
    // 基地护甲
    uint8_t base_state;
    // 机器人自身信息
    uint8_t robot_id;
    uint16_t remain_HP;
    uint16_t max_HP;
    // 剩余弹量与金币
    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;
    // rfid
    uint32_t rfid_status;
    // 小地图
    float x;
		float y;
		uint8_t key;
    // 包尾
    uint8_t _EOF;
}
RefereeDataFrame_SCM_t;

// 23字节
typedef __PACKED_STRUCT
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// 电机数据
		fp32 RightMotorAngle;
		fp32 LeftMotorAngle;
		// imu数据
		uint32_t TimeStamp;
		fp32 q0;
    fp32 q1;
    fp32 q2;
    fp32 q3;
    // 包尾
    uint8_t _EOF;
}
GimabalDataFrame_SCM_t;

// 15字节
typedef __PACKED_STRUCT
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// nuc控制
    fp32 vx;
    fp32 vy;
    fp32 yaw_imu;
    // 包尾
    uint8_t _EOF;
}
NucControlFrame_SCM_t;

// 23字节
typedef __PACKED_STRUCT
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
	// imu数据
	uint32_t TimeStamp;
    fp32 q0;
    fp32 q1;
    fp32 q2;
    fp32 q3;
	uint8_t robot_id;
	uint8_t mode;
    // 包尾
    uint8_t _EOF;
}
GimabalImuFrame_SCM_t;

// 15字节
typedef __PACKED_STRUCT
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
	// 自瞄状态
	uint8_t AimbotState;
	uint8_t AimbotTarget;
	// 自瞄数据
	fp32 Pitch;
    fp32 Yaw;
	//自瞄目标角速度
    fp32 TargetPitchSpeed;
    fp32 TargetYawSpeed;
		uint16_t target_x;
		uint16_t target_y;
    uint32_t SystemTimer; 
		
    // 包尾
    uint8_t _EOF;
	// 处理后数据
	fp32 PitchRelativeAngle;
    fp32 YawRelativeAngle;
}
AimbotFrame_SCM_t;

extern GimabalImuFrame_SCM_t GimabalImu;
extern AimbotFrame_SCM_t Aimbot_s;
#endif