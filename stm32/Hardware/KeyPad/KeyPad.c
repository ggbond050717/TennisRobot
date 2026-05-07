#include "KeyPad/KeyPad.h"
#include "OS/OS.h"
#include "tim.h"

uint8_t Key_Val[3];

uint8_t Bsp_BTNScan(void)
{
	uint8_t key = 0;

	// 将4个按键 组合到一个字节数据的最低2位
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) == 0)
	{
		key |= 0x01;
	}
	key<<=1;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == 0)
	{
		key |= 0x01;
	}
	key<<=1;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == 0)
	{
		key |= 0x01;
	}
	key<<=1;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == 0)
	{
		key |= 0x01;
	}
	
	if (key != Key_Val[2])     			// 键值变化
  {
    Key_Val[0] = key;         		// 按下键值
    Key_Val[1] = Key_Val[2];    	// 松开键值
    Key_Val[2] = key;         		// 历史键值
  }
  else
  {
    Key_Val[0] = 0;               // 清除键值
    Key_Val[1] = 0;               // 清除键值
  }
	
	return Key_Val[0];
	
}

//uint8_t bspEncoder = Nop, prebspEncoder = 0xff;
//uint8_t per_encoder = 0, encoder = 0;
//void Encoder_scan(void)
//{
//	uint16_t Encoder_number =0;
//	encoder = __HAL_TIM_GET_COUNTER(&htim4);
//	if(prebspEncoder!=bspEncoder)
//	{
//		// 停止定时器
//		HAL_TIM_Encoder_Stop(&htim4,TIM_CHANNEL_ALL); // htim为定时器句柄
//		switch(bspEncoder)
//		{
//			case Nop:
//				htim4.Init.Period = 4096;
//				Encoder_number = 0;
//			break;
//			case CH1_Vset:
//				htim4.Init.Period = 200;
//				Encoder_number = NCPS.CH1.V_Set;
//			break;
//			case CH1_Iset:
//				htim4.Init.Period = 600;
//				Encoder_number = NCPS.CH1.I_Set;
//			break;
//			case CH2_Vset:
//				htim4.Init.Period = 200;
//				Encoder_number = NCPS.CH2.V_Set;
//			break;
//			case CH2_Iset:
//				htim4.Init.Period = 600;
//				Encoder_number = NCPS.CH2.I_Set;
//			break;
//			case Fan:
//				htim4.Init.Period = 1;
//				Encoder_number = NCPS.Fan;
//			break;
//		}
//		prebspEncoder = bspEncoder;
//		HAL_TIM_Base_Init(&htim4);
//		__HAL_TIM_GET_COUNTER(&htim4)  = Encoder_number;
//		HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
//	}
//	else
//	{
//		switch(bspEncoder)
//		{
//			case Nop:
//			break;
//			case CH1_Vset:
//				NCPS.CH1.V_Set = __HAL_TIM_GET_COUNTER(&htim4);
//			break;
//			case CH1_Iset:
//				NCPS.CH1.I_Set = __HAL_TIM_GET_COUNTER(&htim4);;
//			break;
//			case CH2_Vset:
//				NCPS.CH2.V_Set = __HAL_TIM_GET_COUNTER(&htim4);
//			break;
//			case CH2_Iset:
//				NCPS.CH2.I_Set = __HAL_TIM_GET_COUNTER(&htim4);
//			break;
//			case Fan:
//				NCPS.Fan = __HAL_TIM_GET_COUNTER(&htim4);
//				if(NCPS.Fan)
//					FAN_ON();
//				else
//					FAN_OFF();
//				if(encoder != per_encoder)
//					NCPS.state_flag = 1;
//			break;
//		}
//		per_encoder = encoder;
//	}
//}

//void SelectGear(uint8_t Key_num)
//{
//	switch(Key_num)
//	{
//		case 0x04:
//			if(NCPS.CH1.warn)
//			{
//				NCPS.CH1.warn = 0;
//			}
//			else
//			{
//				NCPS.CH1.Switch = ~NCPS.CH1.Switch;
//				if(NCPS.CH1.Switch)
//					CH1_ON();
//				else
//					CH1_OFF();
//			}
//			NCPS.state_flag = 2;
//		break;
//		case 0x02:
//			if(NCPS.CH2.warn)
//			{
//				NCPS.CH2.warn = 0;
//			}
//			else
//			{
//				NCPS.CH2.Switch = ~NCPS.CH2.Switch;
//				if(NCPS.CH2.Switch)
//					CH2_ON();
//				else
//					CH2_OFF();
//			}
//			NCPS.state_flag = 3;
//		break;
//		case 0x01:
//			switch(bspEncoder)
//			{
//				case Nop:
//					bspEncoder = CH1_Vset;
//					NCPS.select_flag = 1;
//				break;
//				case CH1_Vset:
//					bspEncoder = CH1_Iset;
//					NCPS.select_flag = 2;
//				break;
//				case CH1_Iset:
//					bspEncoder = CH2_Vset;
//					NCPS.select_flag = 3;
//				break;
//				case CH2_Vset:
//					bspEncoder = CH2_Iset;
//					NCPS.select_flag = 4;
//				break;
//				case CH2_Iset:
//					bspEncoder = Fan;
//					NCPS.select_flag = 0;
//				break;
//				case Fan:
//					bspEncoder = Nop;
//					NCPS.select_flag = 0;
//				break;
//			}
//			NCPS.state_flag = 4;
//		break;
//	}
//}

void KeyPad_Scan(void)
{
	uint8_t Key_num = 0;
	Key_num = Bsp_BTNScan();
//	SelectGear(Key_num);
//	Encoder_scan();
}
