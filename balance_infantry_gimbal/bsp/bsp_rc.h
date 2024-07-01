#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"
#include "main.h"

#define JOINT_RX_BUF_NUM 156u
#define JOINT_FRAME_LENGTH 78u

extern uint8_t sbus_rx_buf1[2][JOINT_RX_BUF_NUM],sbus_rx_buf6[2][JOINT_RX_BUF_NUM];

extern void RC_init1(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_init6(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
