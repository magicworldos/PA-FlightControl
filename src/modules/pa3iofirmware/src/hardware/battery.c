/*
 * battery.c
 *
 *  Created on: Jul 7, 2018
 *      Author: lidq
 */

#include <battery.h>

#define Rheostat_ADC_IRQ ADC_IRQn
#define Rheostat_ADC_INT_FUNCTION ADC_IRQHandler
#define RHEOSTAT_ADC ADC1
#define RHEOSTAT_ADC_CLK RCC_APB2Periph_ADC1
#define RHEOSTAT_ADC_CHANNEL ADC_Channel_13

void battery_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//连续多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换不受外界决定
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//扫描通道数
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5);	//通道X,采样时间为1.5周期,1代表规则通道第1个这个1是啥意思我不太清楚只有是1的时候我的ADC才正常。
	ADC_Cmd(ADC1, ENABLE);	//使能或者失能指定的ADC
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//使能或者失能指定的ADC的软件转换启动功能
}

float battery_read(void)
{
	u16 adc = 0;
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
	{
		//检查制定ADC标志位置1与否 ADC_FLAG_EOC 转换结束标志位
	}
	adc = ADC_GetConversionValue(ADC1);

	return ((float) adc) / 3675.0 * 60.0;
}

