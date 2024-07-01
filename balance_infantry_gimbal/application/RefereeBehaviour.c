/*
* 本部分增添FIFO解决CAN自身FIFO被塞满的情形
* 实际测试不能连发三包以上的CAN，不然后面的会塞不进FIFO
*/
#include "RefereeBehaviour.h"
#include "InterruptService.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "bsp_referee.h"


//Queue_t SendBuffer[2];

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

remote_control_t TuRc;

//ext_game_state_t game_state;
//ext_game_result_t game_result;
//ext_game_robot_HP_t game_robot_HP_t;

//ext_event_data_t field_event;
//ext_supply_projectile_action_t supply_projectile_action_t;
//ext_referee_warning_t referee_warning_t;


//ext_game_robot_status_t robot_state;
//ext_power_heat_data_t power_heat_data_t;
//ext_game_robot_pos_t game_robot_pos_t;
//ext_buff_musk_t buff_musk_t;
//aerial_robot_energy_t robot_energy_t;
//ext_robot_hurt_t robot_hurt_t;
//ext_shoot_data_t shoot_data_t;
//ext_bullet_remaining_t bullet_remaining_t;
//ext_rfid_status_t rfid_status_t;
//ext_student_interactive_data_t student_interactive_data_t;

//ext_dart_client_cmd_t dart_client_cmd_t;
//ext_dart_remaining_time_t dart_remaining_time_t;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

	memset(&TuRc, 0, sizeof(TuRc));
	
//    memset(&game_state, 0, sizeof(ext_game_state_t));
//    memset(&game_result, 0, sizeof(ext_game_result_t));
//    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


//    memset(&field_event, 0, sizeof(ext_event_data_t));
//    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
//    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


//    memset(&robot_state, 0, sizeof(ext_game_robot_status_t));
//    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
//    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
//    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
//    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
//    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
//    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
//    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
//	  memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));


//    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

	if (cmd_id == 0x304)
	{
		TuRc.KeyFormerChannal = TuRc.key.v;
		
		memcpy(&TuRc.mouse , frame + index, 12 );
		uint16_t tmp;
		tmp = TuRc.mouse.x;
		TuRc.mouse.x = - TuRc.mouse.y;
		TuRc.mouse.y = - tmp;
		
		TuRc.KeyJumpChannal = (TuRc.key.v ^ TuRc.KeyFormerChannal);
		RemoteOfflineCounterUpdate();
	}
	
//    switch (cmd_id)
//    {
//        case GAME_STATE_CMD_ID:
//        {
//            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
//						EnQueue(&SendBuffer[0], send_game_status(),11);
//        }
//        break;
//        case GAME_RESULT_CMD_ID:
//        {
//            memcpy(&game_result, frame + index, sizeof(game_result));
//        }
//        break;
//        case GAME_ROBOT_HP_CMD_ID:
//        {
//            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
//						EnQueue(&SendBuffer[1], send_enemy_information(),11);
//        }
//        break;
//        case FIELD_EVENTS_CMD_ID:
//        {
//            memcpy(&field_event, frame + index, sizeof(field_event));
//        }
//        break;
//        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
//        {
//            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
//        }
//        break;
//        case REFEREE_WARNING_CMD_ID:
//        {
//            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
//        }
//        break;
//				case DART_FIRE_COUNTDOWN_CMD_ID:
//        {
//            memcpy(&dart_remaining_time_t, frame + index, sizeof(ext_dart_remaining_time_t));
//        }
//        break;

//        case ROBOT_STATE_CMD_ID:
//        {
//            memcpy(&robot_state, frame + index, sizeof(robot_state));
//						EnQueue(&SendBuffer[1], send_bullet_limit(),11);
//						EnQueue(&SendBuffer[1], send_robot_information(),11);
//        }
//        break;
//        case POWER_HEAT_DATA_CMD_ID:
//        {
//            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
//						EnQueue(&SendBuffer[0], send_power_heat_data(),22);
//        }
//        break;
//        case ROBOT_POS_CMD_ID:
//        {
//            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
//        }
//        break;
//        case BUFF_MUSK_CMD_ID:
//        {
//            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
//        }
//        break;
//        case AERIAL_ROBOT_ENERGY_CMD_ID:
//        {
//            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
//        }
//        break;
//        case ROBOT_HURT_CMD_ID:
//        {
//            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
//        }
//        break;
//        case SHOOT_DATA_CMD_ID:
//        {
//            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
//			EnQueue(&SendBuffer[0], send_bullet_speed(),11);
//        }
//        break;
//        case BULLET_REMAINING_CMD_ID:
//        {
//            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
//        }
//        break;
//		case ROBOT_RFID_STATUS_ID:
//		{
//				memcpy(&rfid_status_t, frame + index, sizeof(ext_rfid_status_t));
//		}
//				case DART_CLIENT_CMD_ID:
//        {
//            memcpy(&dart_client_cmd_t, frame + index, sizeof(ext_dart_client_cmd_t));
//        }
//        break;
//        case STUDENT_INTERACTIVE_DATA_CMD_ID:
//        {
//            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
//        }
//        break;
//        default:
//        {
//            break;
//        }
//    }
}

remote_control_t *get_remotetu_control_point(void)
{
    return &TuRc;
}

/*-----------------------------------------发送FIFO---------------------------------------*/
void QueueInit(Queue_t *q)
{
    q->front = 0;
    q->rear = -1;
    q->counter = 0;
} 


uint8_t IsFull(Queue_t *q)
{
	if(q->counter >= SIZE)
    return 1;
	return 0;
}

int8_t RW_Lock=0;//读写锁

void EnQueue(Queue_t *q, uint8_t *val,uint8_t lenth)
{
	RW_Lock = 1;
	uint8_t i = 0;
    if(IsFull(q))
        return;
		if(q->counter > (SIZE - 11))
			return;
	
	for(;i<lenth;i++)
	{
		q->rear = (q->rear + 1) % SIZE;
		q->arr[q->rear] = *(val+i);
		q->counter++;
	}
	RW_Lock = 0;
}


uint8_t IsEmpty(Queue_t *q)
{
	if(q->counter <= 0)
		return 1;
	return 0;
}

//0代表空 -1代表出错
int Pop(Queue_t *buffer1,Queue_t *buffer2,uint8_t data[11])
{
	if(RW_Lock)
		return -1;
	
	int i = 0;
	if(IsEmpty(buffer1))
	{
		if(IsEmpty(buffer2))
			return 0;
		else if(buffer2->counter %11 == 0)
		{
			for(;i<11;i++)
			{
				data[i] = buffer2->arr[buffer2->front];
				buffer2->front = (buffer2->front + 1) % SIZE;
				buffer2->counter--;
			}
		}
		else
		{
			return -1;
		}
			
	}
	else if(buffer1->counter %11 == 0)
	{
		for(;i<11;i++)
			{
				data[i] = buffer1->arr[buffer1->front];
				buffer1->front = (buffer1->front + 1) % SIZE;
				buffer1->counter--;
			}
	}
	else
	{
		return -1;
	}
	return 1;
}
