#include "ADC_cs/ADC_cs.h"
#include "OS/OS.h"
#include "adc.h"

uint16_t ADC_Value[3] = {0};

void ADC_Value_Average(void)
{
	uint8_t i = 0;
	uint16_t ADC_Total_Value[3] = {0};
	uint16_t ADC_Average_Value[3] = {0};
	for(i = 0; i < 10; i++)
	{
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Total_Value,3);
		HAL_Delay(1);
		ADC_Average_Value[0] += ADC_Total_Value[0];
		ADC_Average_Value[1] += ADC_Total_Value[1];
		ADC_Average_Value[2] += ADC_Total_Value[2];
	}
	ADC_Value[0] = ADC_Average_Value[0] / 10;
	ADC_Value[1] = ADC_Average_Value[1] / 10;
	ADC_Value[2] = ADC_Average_Value[2] / 10;


//	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Value,7);
	HAL_Delay(1);
	HAL_ADC_Stop_DMA(&hadc1);
}
//0.V        			
//1.I						val*3300/4096/0.020(R)/100		单位1mA
//2.VREF2.5V			

void ADC_Update_data(void)
{
	os.Vref2V5 = ADC_Value[2] * 0.8056;
	if((os.Vref2V5>2400)&(os.Vref2V5<2600))  //基准电压在合理范围内，则认为电压基准芯片工作正常，则进行校正
	{
		os.Error_Ratio = 2500000 / os.Vref2V5;
	}
	os.V = ADC_Value[0] * 1.698 * os.Error_Ratio / 1000.0;
	os.I = ADC_Value[1] * 0.4028 * os.Error_Ratio / 1000.0;
	os.P = os.V * os.I / 1000.0;
	
}



