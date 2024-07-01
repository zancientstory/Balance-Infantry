#include "CalculateThread.h"
#include "AttitudeThread.h"
#include "InterruptService.h"
#include "Remote.h"
#include "AimbotCan.h"
#include "user_lib.h"
#include "pid.h"
#include "Motor.h"
#include "RefereeCan.h"
#include "tim.h"
#include "bsp_can.h"
//#include "stdio.h"
#include "loop_fifo.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include "math.h"
#include "Infantry4KeyMap.h"
#include "Setting.h"
#include "td.h"
#include PARAMETER_FILE
#include KEYMAP_FILE
#include "usb.h"
//#define printf(...)  HAL_UART_Transmit_DMA(&huart6,\
//																				(uint8_t  *)u1_buf,\
//																				sprintf((char*)u1_buf,__VA_ARGS__))
//uint8_t u1_buf[30];

Gimbal_t                Gimbal;//云台状态结构
Chassis_t               Chassis;//底盘状态
RC_ctrl_t               Remote;//遥控器数据
AimbotFrame_SCM_t         Aimbot;//自瞄数据
OfflineMonitor_t        Offline;//离线检测结构体
RefereeInformation_t    Referee;//裁判系统数据

first_order_filter_type_t  pitch_aimbot_filter;
td_type_def td_aimepeed;
fp32 pitch_aimbot_filter_param = 0.10f;
fp32 ecd_count=0;
int ammo_left=AMMO_SPEEDSET_30MS_LEFT,ammo_right=AMMO_SPEEDSET_30MS_RIGHT;

void GimbalStateMachineUpdate(void);
void ChassisStateMachineUpdate(void);
void GimbalControlModeUpdate(void);
void GimbalFireModeUpdate(void);
void SetGimbalDisable(void);
void GimbalPIDUpdate(void);
void RotorPIDUpdate(void);
void AmmoPIDUpdate(void);
void GimbalMeasureUpdate(void);
void GimbalCommandUpdate(void);
void ChassisCommandUpdate(void);
void RotorCommandUpdate(void);
void AmmoCommandUpdate(void);
void DebugLEDShow(void);

void BoomBayCover(void);


//int dafu_flag = 0;


uint16_t single_shoot_flag=0;//单发开关
bool_t auto_fire_flag=1;//自动开火开关
bool_t switch_flag=0;//打符切换开关
bool_t enable_flag=0;
int16_t dealta_heat=0;
int32_t onelasttime=0;
int16_t onelastheat=0;
uint16_t count=0;

int32_t    gimbal_init_countdown = 0;          //  云台初始化倒计时器
int32_t    gimbal_fire_countdown = 0;          //  云台射击拨盘转动倒计时器
int32_t    gimbal_lagging_counter = 0;         //  云台堵转计数器




fp32 LimitNormalization(fp32 input);
extern ImuPacketNormal_t ImuPacket;
extern ImuPacketMini_t ImuPackageMini;
int32_t minus = 0;


void CalculateThread(void const * pvParameters)
{
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);//左右摩擦轮pid初始化
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64);//自瞄数据fifo初始化
    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);//滤波器初始化
		TD_init(&td_aimepeed,50,0.01f);
    while(1)
    {
        Remote = *get_remote_control_point();//更新遥控器数据
        Aimbot=*get_usb_aimbot_command_point();//获取自瞄指令
        GetRefereeInformation(&Referee);//获取裁判系统信息 包括枪口的限制
        DeviceOfflineMonitorUpdate(&Offline);//获取模块离线信息
        
        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle);//陀螺仪数据入栈
        
        GimbalStateMachineUpdate();//根据遥控器拨杆决定当前状态（无力，初始化，测试，比赛）
        ChassisStateMachineUpdate();//底盘状态改变
        GimbalControlModeUpdate();//控制权
        GimbalFireModeUpdate();//开火状态转换
        GimbalPIDUpdate();//云台pid重装载
        RotorPIDUpdate();//拨盘pid重装载
        AmmoPIDUpdate();//射击pid重装载
        GimbalMeasureUpdate();//获取电机和imu数据
        GimbalCommandUpdate();//指令的转换
        ChassisCommandUpdate();//底盘指令转换
        RotorCommandUpdate();//拨盘控制转换
        AmmoCommandUpdate();//发射部分控制转化

        
        DebugLEDShow();
        BoomBayCover();//弹舱盖开合
				
				minus = Aimbot.SystemTimer - ImuPacket.TimeStamp;
				

				GimbalMotorControl( Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION ,
                            Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION, 
                            Gimbal.Output.Rotor, //Gimbal.Output.Rotor
                            Gimbal.Output.AmmoLeft, 
                            Gimbal.Output.AmmoRight
                        );
				osDelay(1);
    }
}
uint8_t cflag;
void GimbalStateMachineUpdate(void)
{
		// 电机离线保护
    if(Offline.PitchMotor==DEVICE_OFFLINE||Offline.YawMotor == DEVICE_OFFLINE)
		{
        if(Gimbal.StateMachine!=GM_NO_FORCE)
						Gimbal.StateMachine = GM_NO_FORCE;
        return;
    }
    // 遥控器离线保护
    if(Offline.Remote==DEVICE_OFFLINE)
		{
        if(Gimbal.StateMachine!=GM_NO_FORCE)
						Gimbal.StateMachine=GM_NO_FORCE;
        return;
    }
		if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_G))
				enable_flag^=1;
		
		cflag=Remote.rc.s[0];
		if(enable_flag==1) cflag=1;
    // 云台状态机
    switch (cflag)
		{
        // 右拨杆打到最上，云台复位后进入比赛模式，该模式下开摩擦轮
        case RC_SW_UP:
            if (Gimbal.StateMachine == GM_NO_FORCE)
						{
                Gimbal.StateMachine = GM_INIT;
                gimbal_init_countdown = 1000;
            }
            else if (Gimbal.StateMachine == GM_INIT)
            {
                if (gimbal_init_countdown > 0){
                    gimbal_init_countdown--;
                }
                else{
                    Gimbal.StateMachine = GM_MATCH;//比赛模式
                }
            }
            else{
                Gimbal.StateMachine = GM_MATCH;
            }
            break;
        
        // 右拨杆打到中间，云台复位后进入调试模式
        case RC_SW_MID:
            if (Gimbal.StateMachine == GM_NO_FORCE){
                Gimbal.StateMachine = GM_INIT;
                gimbal_init_countdown = 1000;
            }
            else if (Gimbal.StateMachine == GM_INIT)
            {
                if (gimbal_init_countdown > 0){
                    gimbal_init_countdown--;
                }
                else{
                    Gimbal.StateMachine = GM_TEST;
                }
            }
            else{
                Gimbal.StateMachine = GM_TEST;
            }
            break;
            
        // 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
        case RC_SW_DOWN:
            if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
            break;
        default:
            if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
            break;
    }
}

void ChassisStateMachineUpdate(void)
{
		/*if(Referee.Realtime.ChassisBufferEnergy<5&&!Offline.RefereePowerHeatNode0)
		{
				Chassis.ChassisState=CHASSIS_NO_FORCE;
				Gimbal.StateMachine=GM_INIT;
				return;
		}*/
    //if ((Gimbal.StateMachine == GM_NO_FORCE)  ||  (Gimbal.StateMachine == GM_INIT)) {
		if((Gimbal.StateMachine==GM_NO_FORCE))
        Chassis.ChassisState=CHASSIS_NO_FORCE;//云台无力，底盘强制进入无力状态
		if(Gimbal.StateMachine==GM_INIT)
		{
			 if(Remote.rc.s[1]==2)
					Chassis.ChassisState=CHASSIS_NO_FORCE;
			 else
					Chassis.ChassisState=CHASSIS_NO_FORCE;
		}
    if(Gimbal.StateMachine==GM_TEST||Gimbal.StateMachine==GM_MATCH)
		{
        if(Remote.rc.s[1]==2||enable_flag==1/*||Remote.rc.s[1]==1*/)
				{//左侧拨杆在最下面是底盘有力
            if(CHASSIS_ROTATE_SWITCH_KEYMAP)//小陀螺模式
                Chassis.ChassisState=CHASSIS_ROTATE;
						else 
								if(CHASSIS_STOP_KEYMAP)
										Chassis.ChassisState=CHASSIS_NO_MOVE;
								else
										Chassis.ChassisState=CHASSIS_FOLLOW;												
            if(CHASSIS_HIGH_SPEED_KEYMAP)
                Chassis.ChassisSpeed=CHASSIS_FAST_SPEED;
            else
                Chassis.ChassisSpeed=CHASSIS_NORMAL_SPEED;
						if((RemoteDial() == 1.0f&&Remote.rc.s[1]==2&&Remote.rc.s[0]==3)||CheakKeyPress(KEY_PRESSED_OFFSET_R))
								Chassis.ChassisState=CHASSIS_JUMP;
        }
        else
            Chassis.ChassisState=CHASSIS_NO_FORCE;
    }
}

void SetGimbalDisable(void)
{
    Gimbal.StateMachine = GM_NO_FORCE;
    Gimbal.ControlMode = GM_NO_CONTROL;
    Gimbal.FireMode = GM_FIRE_UNABLE;
}


void GimbalControlModeUpdate(void)
{
    // 比赛模式下
    if(Gimbal.StateMachine==GM_MATCH||Gimbal.StateMachine==GM_TEST)
		{
        // 如果按下鼠标右键并且视觉发现目标，进入自瞄控制
        if(((MousePressRight())||(Remote.rc.s[1]==RC_SW_UP))&&(Offline.AimbotDataNode == DEVICE_ONLINE)&&(Aimbot.AimbotState&AIMBOT_TARGET_INSIDE_OFFSET)||single_shoot_flag)
				{
            if(single_shoot_flag)
								Gimbal.ControlMode = GM_AIMBOT_RUNES;
						else
								Gimbal.ControlMode = GM_AIMBOT_OPERATE;
				}
        else
            Gimbal.ControlMode = GM_MANUAL_OPERATE;//手动状态
    }	
    if(Gimbal.StateMachine==GM_INIT)
        Gimbal.ControlMode=GM_RESET_POSITION;
    if(Gimbal.StateMachine==GM_NO_CONTROL)
        Gimbal.ControlMode=GM_NO_CONTROL;
}


// qylann: 以下代码太乱

void GimbalFireModeUpdate(void)
{		
		//大小符切换
//		if(SUPER_CAP_SWITCH_KEYMAP)
//				switch_flag=(switch_flag+1)%2;
	
    //自动开火开关
//	  if(FIRE_MODE_KEYMAP) 
//        auto_fire_flag=(auto_fire_flag+1)%2;   
	
		//单发开关
		if(SINGLE_SHOOT_KEMAP)
				single_shoot_flag=(single_shoot_flag+1)%3;	
    
		dealta_heat=Referee.Ammo0Limit.Heat-Referee.Realtime.Ammo0Heat;
		if(GetSystemTimer()-onelasttime>=1000)
		{
				
				onelasttime=GetSystemTimer(),
				onelastheat=dealta_heat,
				count=0; 
		}

		if(Gimbal.StateMachine!=GM_MATCH)
		{
				rotor_flag=0,
				ecd_count=RotorMotorMeasure.ecd/10000.0;
				Gimbal.FireMode=GM_FIRE_UNABLE;
				gimbal_fire_countdown=0;
		}
    if(Gimbal.StateMachine==GM_MATCH) 
		{
        if(Gimbal.FireMode==GM_FIRE_UNABLE)
            Gimbal.FireMode=GM_FIRE_READY;
        if (Gimbal.FireMode==GM_FIRE_READY) 
				{		
						//rotor_flag=0,ecd_count=RotorMotorMeasure.ecd/10000.0;
						if(count==0)	rotor_flag=0,ecd_count=RotorMotorMeasure.ecd/10000.0;
						if(((MousePressLeft()||(RemoteDial()==1.0f&&Gimbal.ControlMode==GM_MANUAL_OPERATE))//收到操作手发弹指令
							||(((Gimbal.ControlMode==GM_AIMBOT_OPERATE||Gimbal.ControlMode==GM_AIMBOT_RUNES)&&(((int)(Aimbot.AimbotState)&0x02)!=0))&&(MousePressRight()||(RemoteDial() == 1.0f))))//且视觉开火允许/无自动开火
									&&((count<=Referee.Ammo0Limit.Cooling+onelastheat&&dealta_heat>10)||Referee.Ammo0Limit.Heat==0xFFFF||Referee.Ammo0Limit.Heat==0x0)	)//且热量闭环允许 
						{
								ecd_count+=0.8192*36/8;	
								Gimbal.FireMode=GM_FIRE_BUSY;				
								if(Gimbal.ControlMode==GM_AIMBOT_RUNES)
										gimbal_fire_countdown=ROTOR_TIMESET_BUSY_SINGLE; 
								else
										gimbal_fire_countdown=ROTOR_TIMESET_BUSY; 
								count++;
						}
						else
								gimbal_fire_countdown=-1;
        }
				if(Gimbal.FireMode==GM_FIRE_BUSY&&(gimbal_fire_countdown<=0))
				{
						if(single_shoot_flag!=0||Offline.RefereeAmmoLimitNode0==1)
								gimbal_fire_countdown=200;
						else 
								gimbal_fire_countdown=(int)(10000.0/(dealta_heat/1.5+Referee.Ammo0Limit.Cooling/1.2+5));
						if(Gimbal.ControlMode==GM_AIMBOT_OPERATE&&((int)(Aimbot.AimbotState)&0x02)!=0)
								gimbal_fire_countdown=(int)(10000.0/(dealta_heat/1.1+Referee.Ammo0Limit.Cooling/1.1+5));
						Gimbal.FireMode=GM_FIRE_COOLING;
				}

				if(Gimbal.FireMode==GM_FIRE_COOLING&&gimbal_fire_countdown<=0) 
						Gimbal.FireMode=GM_FIRE_READY;    
				
        //  异常射击模式的状态机，用于反堵转
				if(Gimbal.FireMode==GM_FIRE_LAGGING)
				{
						if(gimbal_fire_countdown<=0)
								Gimbal.FireMode=GM_FIRE_READY,  
								rotor_flag=0,
								ecd_count=RotorMotorMeasure.ecd/10000.0;
				}
				else
				{
						if(Gimbal.Pid.Rotor.s_set-Gimbal.Pid.Rotor.s_fdb>1&&Gimbal.Pid.Rotor.v_fdb<=0)
								gimbal_lagging_counter++;
						else
								gimbal_lagging_counter=0;      
						if (gimbal_lagging_counter>ROTOR_LAGGING_COUNTER_MAX)//ROTOR_LAGGING_COUNTER_MAX
						{        
								gimbal_lagging_counter=0;
								gimbal_fire_countdown=ROTOR_TIMESET_RESERVE;
								Gimbal.FireMode=GM_FIRE_LAGGING;
						}
				}				
				gimbal_fire_countdown--;
		}
}

// qylann: "     "


GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;

void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;
    
    if (CMthis == CMlast){
        return;
    }
    
    
    //  
    
    if (CMthis == GM_MANUAL_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_MANUAL_OPERATE, 
                            YAW_SPEED_MANUAL_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_MANUAL_OPERATE, 
                            PITCH_SPEED_MANUAL_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_OPERATE, 
                            YAW_SPEED_AIMBOT_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_OPERATE, 
                            PITCH_SPEED_AIMBOT_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_RUNES){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_RUNES, 
                            YAW_SPEED_AIMBOT_RUNES, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_RUNES, 
                            PITCH_SPEED_AIMBOT_RUNES, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_RESET_POSITION){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_RESET_POSITION, 
                            YAW_SPEED_RESET_POSITION, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_RESET_POSITION, 
                            PITCH_SPEED_RESET_POSITION, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_NO_FORCE, 
                            YAW_SPEED_NO_FORCE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_NO_FORCE, 
                            PITCH_SPEED_NO_FORCE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    
    CMlast = CMthis;
}





GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;
    
    if (FMthis == FMlast){
        return;
    }
    
    //
    
    if ((FMthis == GM_FIRE_READY)  ||  (FMthis == GM_FIRE_COOLING)){
        cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED, ROTOR_FORWARD, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_BUSY){
				if(Gimbal.ControlMode==GM_AIMBOT_RUNES)
						cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED_SINGLE, ROTOR_SINGLE, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
				else	
						cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED, ROTOR_FORWARD, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else if (FMthis == GM_FIRE_LAGGING){
        cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED, ROTOR_SINGLE, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    else{
        cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED, ROTOR_FORWARD, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    }
    
    FMlast = FMthis;
}


uint8_t MSthis = 0;
uint8_t MSlast = 0;
void AmmoPIDUpdate(void)
{
    MSthis = Referee.Ammo0Limit.Speed;//Gimbal.Referee.MaxSpeed;
    
    if (MSthis != MSlast){
        switch (MSthis){
            case 10:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 12:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_12MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_12MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 14:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_14MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_14MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 15:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_15MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_15MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 16:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_16MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_16MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 18:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_18MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_18MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 22:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_22MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_22MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 30:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            default:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, DEFAULT_AMMOL_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, DEFAULT_AMMOR_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
        }
    }
    
    MSlast = MSthis;
}


void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}



int Aimtime1=90,Aimtime2=60,kflag=1;
void GimbalCommandUpdate(void)
{
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE){
        Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc_yaw(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle/*-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0142 */, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw,0);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE || (Gimbal.ControlMode == GM_AIMBOT_RUNES)){
//        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
				if((Aimbot.AimbotState&0x03)!=0)
				{
						
						float aim_yawspeed=td_aimepeed.x1/3.14159f*180.0f;
						float aim_yawaccel=td_aimepeed.x2/3.14159f*180.0f;
						float delaytime1=(-minus+Aimtime1)/1000.0f;	
						if(minus<-1000)		delaytime1=Aimtime2/1000.0f;
						float delaytime2=(-minus+Aimtime1)/1000.0f;	
						if(minus<-1000)		delaytime2=Aimtime2/1000.0f;					
						Gimbal.Command.Yaw = /*LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer+t)) + */Aimbot.Yaw+aim_yawspeed*delaytime1*kflag+aim_yawaccel*delaytime2*delaytime2*0.5f*kflag;
						Gimbal.Command.Pitch = /*LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer+t)) + */Aimbot.Pitch;
						Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
						Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
						Gimbal.Output.Yaw = cascade_PID_calc_yaw(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw,(aim_yawspeed+aim_yawaccel*delaytime2)*kflag);
						Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed+Aimbot.TargetPitchSpeed/3.14159f*180.0f*0.9f, Gimbal.Command.Pitch);					
				}
				else
				{
//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
						Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
						Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;					
						Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
						Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
						Gimbal.Output.Yaw = cascade_PID_calc_yaw(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw,0);
						Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
				}
    } 
    else if (Gimbal.ControlMode == GM_AIMBOT_RUNES){
        
//        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
        Gimbal.Command.Yaw =/* LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + */Aimbot.YawRelativeAngle;
        Gimbal.Command.Pitch =/* LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + */Aimbot.PitchRelativeAngle;
        Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;			
//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        Gimbal.Output.Yaw = cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);

    }
    else if (Gimbal.ControlMode == GM_RESET_POSITION){
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        fp32 YawTempCommand = loop_fp32_constrain(YAW_ZERO_ECDANGLE, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle - 180.0f, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle + 180.0f);
//        Gimbal.Output.Yaw peeeeeeetyuok-p= YAW_MOTOR_DIRECTION * cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed, YAW_ZERO_ECDANGLE);
        Gimbal.Pid.Yaw.v_set = PID_calc(&Gimbal.Pid.Yaw.pid_outside, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, YawTempCommand);
        Gimbal.Output.Yaw = PID_calc(&Gimbal.Pid.Yaw.pid_inside,Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0,Gimbal.Pid.Yaw.v_set);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, 0);       
				pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else{
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Pitch = 0;
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
}

void ChassisCommandUpdate(void)
{
    if ((Chassis.ChassisState == CHASSIS_NO_FORCE) ) {
        Chassis.ChassisCommandX = 0.0f;
        Chassis.ChassisCommandY = 0.0f;
    }
    else {
        Chassis.ChassisCommandX = CHASSIS_CMD_X_KEYMAP;
        Chassis.ChassisCommandY = CHASSIS_CMD_Y_KEYMAP;
    }
}

void RotorCommandUpdate(void)
{
    if(Gimbal.FireMode == GM_FIRE_BUSY||Gimbal.FireMode == GM_FIRE_READY||Gimbal.FireMode == GM_FIRE_COOLING)
		{
        //Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
//				if((Aimbot.AimbotState&0x03)!=0&&Gimbal.ControlMode==GM_AIMBOT_OPERATE)
//				{
//						if(Gimbal.FireMode == GM_FIRE_BUSY)
//								Gimbal.Output.Rotor = 10000;
//						else
//								Gimbal.Output.Rotor = cascade_PID_calc(&Gimbal.Pid.Rotor,0, RotorMotorMeasure.speed_rpm,0);
//				}
//				else
						Gimbal.Output.Rotor = cascade_PID_calc(&Gimbal.Pid.Rotor, RotorMotorMeasure.ecd/10000.0+0.8192*rotor_flag, RotorMotorMeasure.speed_rpm,ecd_count);
//				Gimbal.Output.Rotor=0;
//				if(Gimbal.FireMode == GM_FIRE_BUSY)
//						Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor.pid_inside,RotorMotorMeasure.speed_rpm,ROTOR_SPEEDSET_FORWARD);
//				else
//						Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor.pid_inside,RotorMotorMeasure.speed_rpm,0);
    
    }
    if(Gimbal.FireMode == GM_FIRE_LAGGING)
		{
        //Gimbal.Command.Rotor = ROTOR_SPEEDSET_BACKWARD * (-ROTOR_MOTOR_DIRECTION);
				Gimbal.Output.Rotor = -5000;
    }
    if(Gimbal.FireMode == GM_FIRE_UNABLE)
		{
        //Gimbal.Command.Rotor = 0;
        Gimbal.Output.Rotor = 0;
    }
}






void AmmoCommandUpdate(void)
{
		if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_Q))
				if(ammo_left>6000)
					ammo_left-=100,
					ammo_right-=100;
		if(CheakKeyPressOnce(KEY_PRESSED_OFFSET_E))
				if(ammo_left<8000)
					ammo_left+=100,
					ammo_right+=100;
    if (Gimbal.FireMode == GM_FIRE_UNABLE){
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                0
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                0
                                                );/*			
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = 0;
        Gimbal.Output.AmmoRight = 0;*/
        return;
    }
    switch (MSthis){
        case 10:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_10MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_10MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 12:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_12MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_12MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 14:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_14MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_14MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 15:
			
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_15MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_15MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
			
			
            break;
        
        case 16:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_16MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_16MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 18:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_18MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_18MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 22:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_22MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_22MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 30:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                ammo_left				* AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                ammo_right * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        default:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                ammo_left * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                ammo_right * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
    }
    
}


void GetGimbalMotorOutput(GimbalOutput_t *out)
{
    memcpy(out, &Gimbal.Output, sizeof(GimbalOutput_t));
}

bool_t cover_flag = 0;
void BoomBayCover(void)
{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
		
    if (Gimbal.StateMachine == GM_MATCH) {
        if (cover_flag == 0) {
            cover_flag = 2;
            
        }
        
        if (COVER_SWITCH_KEYMAP) {
            if (cover_flag == 1) {
                cover_flag = 2;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
            }
            else if (cover_flag == 2) {
                cover_flag = 1;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
            }
        }
    }
    else {
        if (SHOOT_COMMAND_KEYMAP) {
              cover_flag = 1;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
        }
		else{
		 cover_flag = 2;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
		}

    }
    
		
   
    if ((cover_flag == 1)  ||  (cover_flag == 0)) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);//四号：2245 三号 1250    打开
     }
    if (cover_flag == 2) {
         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 650);//四号：500   关闭
    }
    
    
    
}
uint8_t dash_flag=0;


void GetGimbalRequestState(GimbalRequestState_t *RequestState)
{
	   if (Gimbal.StateMachine == GM_NO_FORCE) {
         RequestState->GimbalState |= (uint8_t)(1 << 0);
    }
	
	
	
    RequestState->AimbotRequest = 0x00;
    
    
    RequestState->AimbotRequest |= (uint8_t) (1 << 0);
		GimabalImu.mode = 0x00;
    if((Remote.mouse.press_r==PRESS)||(Remote.rc.s[1]==RC_SW_UP))
				GimabalImu.mode |= (uint8_t)(1 << 0);
		GimabalImu.mode |= (uint8_t)(1 << (1+single_shoot_flag));		
    RequestState->ChassisMoveXRequest = Chassis.ChassisCommandX * 32767;
    RequestState->ChassisMoveYRequest = Chassis.ChassisCommandY * 32767;
    RequestState->ChassisStateRequest = 0x00;
		if(CHASSIS_DASH_KEYMAP)
				dash_flag=(dash_flag+1)%2;
    
    
    if (Chassis.ChassisState != CHASSIS_NO_FORCE) {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 1);
        // 运动状态
        if (Chassis.ChassisState == CHASSIS_NO_MOVE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 2);
        }
        else if (Chassis.ChassisState == CHASSIS_FOLLOW) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 3);
        }
        else if (Chassis.ChassisState == CHASSIS_ROTATE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 4);
        }
         else if (Chassis.ChassisState == CHASSIS_JUMP) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 6);
        }      
				if(CHASSIS_HIGH_SPEED_KEYMAP){
					RequestState->ChassisStateRequest |= (uint8_t)(1<<5);
				}
				if(CHASSIS_STOP_KEYMAP){
					RequestState->ChassisStateRequest |= (uint8_t)(1<<2);
				}
//				if(!switch_flag){
//					RequestState->ChassisStateRequest |= (uint8_t)(1<<7);
//				}
				if(dash_flag)
					 RequestState->ChassisStateRequest |= (uint8_t)(1 << 7);
				
					
        
//        if (Chassis.ChassisSpeed = CHASSIS_NORMAL_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 5);
//        }
//        else if (Chassis.ChassisSpeed == CHASSIS_FAST_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 6);
//        }
//        else if (Chassis.ChassisSpeed == CHASSIS_LOW_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 7);
//        }
    }
    else {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 0);
    }
    
    RequestState->GimbalState = 0x00;
    
			
    
    if ((Remote.mouse.press_r == PRESS)  ||  (Remote.rc.s[1] == RC_SW_UP)) {
        RequestState->GimbalState |= (uint8_t) (1 << 1);
    }
    
    if (cover_flag == 2) {
        RequestState->GimbalState |= (uint8_t) (1 << 3);
    }
    else if (cover_flag == 1) {
        RequestState->GimbalState |= (uint8_t) (1 << 4);
    }
	
		if(auto_fire_flag == 1){
			RequestState->GimbalState |= (uint8_t) (1 << 6);
		}
		if(single_shoot_flag == 1)
		{
				//if(switch_flag)
				//		RequestState->AimbotRequest |= (uint8_t)(1 << 4);
				//else
						RequestState->AimbotRequest |= (uint8_t)(1 << 5);
		}    
		else
				RequestState->AimbotRequest |= (uint8_t) (1 << 0);
    
        
    GimabalImu.robot_id=Referee.RobotState.RobotID;
    
    RequestState->Reserve = 0x00;
    
}



void DebugLEDShow(void)
{
    if (Offline.AimbotDataNode == DEVICE_ONLINE){
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        if ((Aimbot.AimbotState & AIMBOT_TARGET_INSIDE_OFFSET)){
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
    }
    else{
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    
    
}


fp32 LimitNormalization(fp32 input)
{
    if (input > 1.0f){
        return 1.0f;
    }
    else if (input < -1.0f){
        return -1.0f;
    }
    else{
        return input;
    }
}



void RefereeHeatInterpolation(void)
{
    Referee.Realtime.Ammo0Heat -= Referee.Ammo0Limit.Cooling / 10;
    if (Referee.Realtime.Ammo0Heat < 0) {
        Referee.Realtime.Ammo0Heat = 0;
    }
}



