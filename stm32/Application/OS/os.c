#include "OS/os.h"
#include "stdio.h"
#include "string.h"
#include "adc.h"
#include "tim.h"
#include "dma.h"
#include "usart.h"
#include "PID/pid.h"
#include "Motor/Motor.h"
#include "Serial/serial.h"
#include "LCD/LCD_init.h"
#include "LCD/LCD_dma.h"
#include "GUI/GUI.h"
#include "ADC_cs/ADC_cs.h"
#include "KeyPad/KeyPad.h"



os_struct os;


void _os(void)
{
	uint8_t state = 0;
	HAL_ADCEx_Calibration_Start(&hadc1);
	Motor_PWM_Init();
	Enconder_Init();
	LCD_Init();
	HAL_Delay(1);
	LCD_Show_Interface();
	PID_Init(&Left_PID);
	PID_Init(&Right_PID);
	PID_Init(&Roller_PID);
//	Motor_PWM(1000,1000,1000);
	usart2_rx_buffer = usart2_rx_buffer1;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2_rx_buffer,BUFF_SIZE);	//手动开启串口DMA模式接收数据
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);		   	//手动关闭DMA_IT_HT中断
	
	
	HAL_TIM_Base_Start_IT(&htim7);
	
	
	while(1)
	{
		if(os.cnt_10)
		{
			if((os.target_LeftSpeed > 0) && (os.target_RightSpeed < 0))
			{
				serial_communication(os.LeftSpeed_sum * 0.75,-os.RightSpeed_sum);
			}
			else if((os.target_LeftSpeed < 0) && (os.target_RightSpeed > 0))
			{
				serial_communication(os.LeftSpeed_sum * 0.75,-os.RightSpeed_sum);
			}
			else
			{
				serial_communication(os.LeftSpeed_sum,-os.RightSpeed_sum);
			}
//			Bsp_BTNScan();
			LCD_Refresh();
			os.cnt_10 = 0;
			os.LeftSpeed_sum = 0;
			os.RightSpeed_sum = 0;
		}
		switch(state)
		{
			case 0:
				state = 1;
			break;
			case 1:
				if(usart2_rx_flag)
				{
					process_uart_data(usart2_rx_buffer);
					usart2_rx_flag = 0;
				}
				state = 2;
			break;
			case 2:
				ADC_Value_Average();
				state = 3;
			break;
			case 3:
				ADC_Update_data();
				state = 4;
			break;
			case 4:
//			printf("target_leftspeed=%d,target_rightspeed=%d\r\n",os.target_LeftSpeed,os.target_RightSpeed);
//			printf("key=%d,encode=%d\r\n",Key_Val[0],__HAL_TIM_GET_COUNTER(&htim8));
//			printf("CH4=%d,CH5=%d,CH14=%d,tim=%d\r\n",ADC_Value[0], ADC_Value[1], ADC_Value[2], os.cnt);
//			printf("V=%d,I=%d,P=%.2f,vref=%d\r\n",os.V, os.I, os.P, os.Vref2V5);
//			printf("left=%d,right=%d,roller=%d,tim=%d\r\n",os.LeftSpeed,os.RightSpeed,os.RollerSpeed,os.cnt);
//			printf("left_pwm=%d,right_pwm=%d,roller_pwm=%d\r\n",(int)Left_PID.PWM,(int)Right_PID.PWM,(int)Roller_PID.PWM);
//			sprintf((char *)usart2_tx_buffer,"left=%d,right=%d,tim=%d\r\n",Motor_Speed_Left,Motor_Speed_Right,tim_cnt);
				state = 5;
			break;
			case 5:
				state = 0;
			break;
			default:
			break;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//定时器回调函数
{

	if(htim == (&htim7))			//10ms
	{
		os.cnt++;
		if(os.cnt % 10 == 0)
		{
			os.cnt_10 = 1;
		}
		os.LeftSpeed = __HAL_TIM_GET_COUNTER(&htim2);
		os.RightSpeed = __HAL_TIM_GET_COUNTER(&htim5);
		os.RollerSpeed = __HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		
		

		if(os.LeftSpeed > 32768)
		{
			os.LeftSpeed = os.LeftSpeed - 65535;
		}
		if(os.RightSpeed > 32768)
		{
			os.RightSpeed = os.RightSpeed - 65535;
		}
		os.RightSpeed = -os.RightSpeed;
		if(os.RollerSpeed > 32768)
		{
			os.RollerSpeed = os.RollerSpeed - 65535;
		}
		
		if((os.target_LeftSpeed > 0) && (os.target_RightSpeed < 0))
		{
			Left_PID.actual_val = os.LeftSpeed * 0.75;
			Right_PID.actual_val = -os.RightSpeed;
		}
		else if((os.target_LeftSpeed < 0) && (os.target_RightSpeed > 0))
		{
			Left_PID.actual_val = os.LeftSpeed;
			Right_PID.actual_val = -os.RightSpeed * 0.75;
		}
		else
		{
			Left_PID.actual_val = os.LeftSpeed;
			Right_PID.actual_val = -os.RightSpeed;
		}
		Roller_PID.actual_val = -os.RollerSpeed;
		os.LeftSpeed_sum += os.LeftSpeed;
		os.RightSpeed_sum += os.RightSpeed;
		Motor_PWM_PID(os.target_LeftSpeed,os.target_RightSpeed,0);
//		Motor_PWM_PID(5,5,15);
	}
}

/* 串口接收完成回调函数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		usart2_rx_flag = 1;
		if(buffer_AB)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer2, BUFF_SIZE);
			usart2_rx_buffer = usart2_rx_buffer1;
		}
		else
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer1, BUFF_SIZE);
			usart2_rx_buffer = usart2_rx_buffer2;
		}
		buffer_AB =! buffer_AB;
		
	}
}

/* 串口错误回调函数 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buffer, BUFF_SIZE);//手动开启串口DMA模式接收数据
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
		memset(usart2_rx_buffer, 0, BUFF_SIZE);							   // 清除接收缓存
	}
}

