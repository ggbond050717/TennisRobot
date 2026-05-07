#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include <stdio.h>

#define PACKET_HEADER 0xAA55
#define BUFF_SIZE	12			    //接收缓存大小

#pragma pack(push, 1)
typedef struct {
    uint16_t header;				//帧头2字节
    uint8_t length;					//长度1字节
    int32_t left_speed;			//左4字节
    int32_t right_speed;		//右4字节
    uint8_t checksum;				//校验和1字节
} WheelPacket;							//总共12字节
#pragma pack(pop)

#define PACKET_SIZE sizeof(WheelPacket)


extern uint8_t usart2_rx_buffer1[BUFF_SIZE];      // 创建接收缓存,大小为BUFF_SIZE
extern uint8_t usart2_rx_buffer2[BUFF_SIZE];      // 创建接收缓存,大小为BUFF_SIZE
extern uint8_t *usart2_rx_buffer;
extern uint8_t buffer_AB;
extern uint8_t usart2_rx_flag;

void serial_communication(int32_t left, int32_t right);
void process_uart_data(uint8_t *rx_buffer);

#endif
