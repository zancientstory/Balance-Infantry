/*
 * @Author: BletillaQi 1370670703@qq.com
 * @Date: 2024-03-14 03:07:17
 * @LastEditors: BletillaQi 1370670703@qq.com
 * @LastEditTime: 2024-03-14 14:14:31
 * @FilePath: \MDK-ARMe:\study\robot\auto_shaobing\Double_Head_Dragon\Gimbal_Mid\Device\Usb.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Usb.h"
#include "UsbPackage.h"

#include "InterruptService.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

GimabalImuFrame_SCM_t GimabalImu;
AimbotFrame_SCM_t Aimbot_s;

extern OfflineMonitor_t OfflineMonitor;

/**
 * @brief          Usb接收数据
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @retval         none
 */
void UsbReceive(uint8_t *rx_data, uint8_t len)
{
	if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF)
	{
		switch (rx_data[1])
		{
		case AIMBOT_DATA_0_ID:
			AimbotDataNodeOfflineCounterUpdate();
			memcpy(&Aimbot_s, rx_data, len);
			//Aimbot_s.SystemTimer=GetSystemTimer();
			Aimbot_s.YawRelativeAngle = Aimbot_s.Yaw;
			Aimbot_s.PitchRelativeAngle = Aimbot_s.Pitch;
			break;
		default:
			break;
		}
	}
}

/**
 * @brief          Usb数据发送
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @param[in]      数据id
 * @retval         none
 */
void UsbSendMessage(uint8_t *address, uint16_t len, uint8_t id)
{
	address[0] = 0x55;
	address[1] = id;
	address[len - 1] = 0xff;
	CDC_Transmit_FS(address, len);
}

/**
 * @brief          获取usb自瞄指针
 * @retval         none
 */
const AimbotFrame_SCM_t *get_usb_aimbot_command_point(void)
{
    return &Aimbot_s;
}