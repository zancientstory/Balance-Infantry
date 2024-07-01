#ifndef SETTING_H
#define SETTING_H

#define MOTOR_OFFLINE_TIMEMAX   50
#define REMOTE_OFFLINE_TIMEMAX  1000
#define AIMBOT_OFFLINE_TIMEMAX  550
#define REFEREE_OFFLINE_TIMEMAX 3000



//#define HERO1
//#define HERO1_0
//#define HERO1_1
//#define HERO1_2
#define INFANTRY3
//#define INFANTRY4
//#define INFANTRY5
//#define SENTRY7

#ifdef INFANTRY4

// 参数配置文件
#define PARAMETER_FILE "Infantry4Parameter.h"
// 键位配置文件
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu安装方向
#define IMU_DIRECTION_zryx_XYZ
// gyro yaw轴偏置
#define GYRO_YAW_BIAS -0.0055f
// 主发射机构类型
#define MAIN_SHOOTER_TYPE_NORMAL
//#define MAIN_SHOOTER_TYPE_HEAVY
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x202
#define AMMO_RIGHT_MOTOR_ID         0x201
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           -144.102539f
#define PITCH_MIN_ANGLE             -31.0f
#define PITCH_MAX_ANGLE             18.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan1
#endif


#ifdef INFANTRY3

// 参数配置文件
#define PARAMETER_FILE "Infantry3Parameter.h"
// 键位配置文件
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu安装方向
#define IMU_DIRECTION_yrxz_XYZ
// gyro yaw轴偏置
//#define GYRO_YAW_BIAS -0.01008f//-0.01065f
// 主发射机构类型
#define MAIN_SHOOTER_TYPE_NORMAL
//#define MAIN_SHOOTER_TYPE_HEAVY
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       1
#define AMMO_LEFT_MOTOR_DIRECTION   1
#define AMMO_RIGHT_MOTOR_DIRECTION  -1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           -161.0f//21.4f
#define PITCH_MIN_ANGLE             -60.0f//-24.0f//-26.0f
#define PITCH_MAX_ANGLE             60.0f//15.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_30MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_30MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan2
#endif



#ifdef SENTRY7
// 参数配置文件
#define PARAMETER_FILE "Sentry7Parameter.h"
// 键位配置文件
#define KEYMAP_FILE "Sensor7KeyMap.h"
// imu安装方向
#define IMU_DIRECTION_yrxz_XYZ
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         -1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           0.0f
#define PITCH_MIN_ANGLE             -8.8f
#define PITCH_MAX_ANGLE             28.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan2
#endif




#ifdef HERO1_0
// 参数配置文件
#define PARAMETER_FILE "Hero1_0Parameter.h"
// 键位配置文件
#define KEYMAP_FILE "Hero1_0KeyMap.h"
// imu安装方向
#define IMU_DIRECTION_xyz_XYZ
// 安装pitch辅助动力电机
#define PITCH_AUX
// 安装辅助发射机构
#define SHOOTOR_AUX
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x208
//#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// pitch辅助动力电机
// 此电机与pitch轴主电机安装方向相反，执行力矩相同，不关注其编码器
#define PITCH_AUX_MOTOR_ID          0x207
// 辅助发射机构拨盘电机
// 此电机与pitch轴主电机安装方向相反，执行力矩相同，不关注其编码器
//#define ROTOR_AUX_MOTOR_ID          0x203
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   1
#define AMMO_RIGHT_MOTOR_DIRECTION  -1
// 辅助发射机构拨盘电机
//#define ROTOR_AUX_MOTOR_DIRECTION   -1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           4.65820313f
#define PITCH_MIN_ANGLE             -35.0f
#define PITCH_MAX_ANGLE             18.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_10MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_10MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_10MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan1


#endif

#ifdef HERO1_1
// 参数配置文件
#define PARAMETER_FILE "Hero1_1Parameter.h"
// 键位配置文件
#define KEYMAP_FILE "Hero1_1KeyMap.h"
// imu安装方向
#define IMU_DIRECTION_xzry_XYZ
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x207
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   1
#define AMMO_RIGHT_MOTOR_DIRECTION  -1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           -114.873047f
#define PITCH_MIN_ANGLE             -36.0f
#define PITCH_MAX_ANGLE             26.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_10MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_10MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_10MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan2


#endif



//#ifdef INFANTRY5

//// 参数配置文件
//#define PARAMETER_FILE "Infantry4Parameter.h"
//// 键位配置文件
//#define KEYMAP_FILE "Infantry4KeyMap.h"
//// imu安装方向
//#define IMU_DIRECTION_xyz_XYZ
//// 电机ID分配
//#define YAW_MOTOR_ID                0x205
//#define PITCH_MOTOR_ID              0x206
//#define ROTOR_MOTOR_ID              0x203
//#define AMMO_LEFT_MOTOR_ID          0x201
//#define AMMO_RIGHT_MOTOR_ID         0x202
//// 电机安装方向
//// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
//// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
//// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
//#define YAW_MOTOR_DIRECTION         1
//#define PITCH_MOTOR_DIRECTION       -1
//#define ROTOR_MOTOR_DIRECTION       1
//#define AMMO_LEFT_MOTOR_DIRECTION   -1
//#define AMMO_RIGHT_MOTOR_DIRECTION  1
//// 云台YAW轴零点和俯仰限幅
//#define YAW_ZERO_ECDANGLE           2.72460938f
//#define PITCH_MIN_ANGLE             -20.0f
//#define PITCH_MAX_ANGLE             20.0f
//// 默认摩擦轮速度
//#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
//#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
//#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
//// 通信can总线位置
//#define COMMUNICATE_CANPORT         hcan2
//#endif

#ifdef INFANTRY5

// 2?êy???????t
#define PARAMETER_FILE "Infantry5Parameter.h"
// ?ü?????????t
#define KEYMAP_FILE "Infantry4KeyMap.h"
// imu°2×°·??ò
#define IMU_DIRECTION_zryx_XYZ
// gyro yaw?á????
#define GYRO_YAW_BIAS -0.0025f//055f
// ?÷·￠é??ú11ààDí
#define MAIN_SHOOTER_TYPE_NORMAL
//#define MAIN_SHOOTER_TYPE_HEAVY
// μ??úID·???
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// μ??ú°2×°·??ò
// ??ì¨μ??ú?y?ò???ˉ·??òoí??ì¨×?ì?￡¨????￡×?±ê?μí??ò?a1￡?·′?ò?a-1
// 2|?ìμ??ú?y?ò???ˉ·??òoíμˉíè??è??11ü·??òí??ò?a1￡?·′?ò?a-1
// ?|2á??μ??ú?y?ò???ˉ·??òoíμˉμàí??ò?a1￡?·′?ò?a-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// ??ì¨YAW?áá?μ?oí????T·ù
#define YAW_ZERO_ECDANGLE           19.0f//-144.102539f
#define PITCH_MIN_ANGLE             -19.5f//-31.0f
#define PITCH_MAX_ANGLE             25.0f//15.0f//18.0f
// ??è??|2á???ù?è
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
// í¨D?can×ü??????
#define COMMUNICATE_CANPORT         hcan1
#endif



#ifdef HERO1_2
// 参数配置文件
#define PARAMETER_FILE "Hero1_1Parameter.h"
// imu安装方向
#define IMU_DIRECTION_xyz_XYZ
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x207
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   1
#define AMMO_RIGHT_MOTOR_DIRECTION  -1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           -164.223633f
#define PITCH_MIN_ANGLE             -36.0f
#define PITCH_MAX_ANGLE             26.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_10MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_10MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_10MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan2


#endif


//#define IMU_DIRECTION_xyz_XYZ
//#define IMU_DIRECTION_yrxz_XYZ
//#define IMU_DIRECTION_rxryz_XYZ
//#define IMU_DIRECTION_ryxz_XYZ

//#define IMU_DIRECTION_zryx_XYZ
//#define IMU_DIRECTION_yzx_XYZ
//#define IMU_DIRECTION_rzyx_XYZ
//#define IMU_DIRECTION_ryrzx_XYZ




#endif

