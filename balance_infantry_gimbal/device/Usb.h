/*
 * @Author: BletillaQi 1370670703@qq.com
 * @Date: 2024-03-14 03:07:35
 * @LastEditors: BletillaQi 1370670703@qq.com
 * @LastEditTime: 2024-03-14 13:59:28
 * @FilePath: \MDK-ARMe:\study\robot\auto_shaobing\Double_Head_Dragon\Gimbal_Mid\Device\Usb.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef USB_H
#define USB_H

#include "struct_typedef.h"
#include "UsbPackage.h"

/**
 * @brief          Usb接收数据
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @retval         none
 */
extern void UsbReceive(uint8_t* rx_data, uint8_t len);

/**
 * @brief          Usb数据发送
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @param[in]      数据id
 * @retval         none
 */
extern void UsbSendMessage(uint8_t* address, uint16_t len, uint8_t id);

/**
 * @brief          获取usb自瞄指针
 * @retval         none
 */
extern const AimbotFrame_SCM_t *get_usb_aimbot_command_point(void);

#endif