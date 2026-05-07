#include "Serial/serial.h"
#include "OS/os.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

uint8_t usart2_rx_buffer1[BUFF_SIZE];      // 创建接收缓存,大小为BUFF_SIZE
uint8_t usart2_rx_buffer2[BUFF_SIZE];      // 创建接收缓存,大小为BUFF_SIZE
uint8_t *usart2_rx_buffer;
uint8_t buffer_AB;
uint8_t usart2_rx_flag = 0;

void debug_format_hex(WheelPacket *packet) {
    char buffer[PACKET_SIZE*3 + 1] = {0};  // 每个字节占3字符（XX+空格）
    uint8_t *p = (uint8_t*)packet;
    
    for(int i=0; i<PACKET_SIZE; i++) {
        sprintf(buffer + i*3, "%02X ", p[i]);  // 格式化到缓冲区[4,7](@ref)
    }
//    printf("Formatted Hex: %s\r\n", buffer);
		HAL_Delay(1);
}

void serial_communication(int32_t left, int32_t right) 
{
	WheelPacket packet;
	// 包头处理（大端转小端存储）
	packet.header = __REV16(PACKET_HEADER); // STM32默认小端存储，需手动转换
	// 数据段长度（固定为9字节）
	packet.length = sizeof(packet) - sizeof(packet.header) - sizeof(packet.checksum); // 固定为9字节
	packet.left_speed = left;  // 使用HAL库内置字节序转换宏
	packet.right_speed = right;

	// 计算校验和（从length字段开始）
	packet.checksum = 0;
	uint8_t* p = (uint8_t*)&packet + 2; // 跳过header的2字节
	for(int i=0; i < PACKET_SIZE-3; i++) {
			packet.checksum ^= p[i];
	}
	// 发送前禁用结构体填充
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&packet, PACKET_SIZE);
	debug_format_hex(&packet);
	
}

/* 数据包解析核心函数 */
void process_uart_data(uint8_t *rx_buffer)
{
	uint16_t processed_idx = 0; // 记录已处理数据的索引
	
	 // 遍历缓冲区查找有效数据包
	while (processed_idx + PACKET_SIZE <= BUFF_SIZE) 
	{
		// 查找包头 (0xAA 0x55)
		if (rx_buffer[processed_idx] != 0xAA || rx_buffer[processed_idx + 1] != 0x55) 
		{
			processed_idx++;
			continue;
		}
			
		// 提取完整数据包
		WheelPacket packet;
		memcpy(&packet, &rx_buffer[processed_idx], PACKET_SIZE);
			
		// 校验数据长度
		if (packet.length != (sizeof(WheelPacket) - 3)) // 2(header)+1(checksum)=3
		{ 
			processed_idx += 2; // 跳过错误包头
			continue;
		}
		
		// 计算校验和（从length字段开始）
		uint8_t calc_checksum = 0;
		uint8_t* p = (uint8_t*)&packet + 2;
		for (int i = 0; i < sizeof(WheelPacket) - 3; i++) 
		{
			calc_checksum ^= p[i];
		}
		
		if (calc_checksum != packet.checksum) 
		{
			processed_idx += 2; // 跳过校验失败包
			continue;
		}
			
		// 字节序转换（小端转CPU序）
		int32_t left = __REV(packet.left_speed);
		int32_t right = __REV(packet.right_speed);
		
		// 更新电机控制
		os.target_LeftSpeed = left;
		os.target_RightSpeed = right;
	
		// 移动处理位置
		processed_idx += PACKET_SIZE;
			
		debug_format_hex(&packet);

	}

}

