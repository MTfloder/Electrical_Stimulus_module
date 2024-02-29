/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHOCK_MODULE_H__
#define __SHOCK_MODULE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

// ��̼������ṹ��	//
typedef struct
{
	// ģ����������
	//GPIO_TypeDef* GPIOx_Boost_L;
	//uint16_t GPIO_Pin_Boost_L;
	TIM_HandleTypeDef* htim_Boost_L;
	uint32_t Channel_Boost_L;
	
	GPIO_TypeDef* GPIOx_Net_P;
	uint16_t GPIO_Pin_Net_P;
	GPIO_TypeDef* GPIOx_Net_N;
	uint16_t GPIO_Pin_Net_N;
	
	// ������Щֵ���۰�ԭ��ͼ��������󲻱�
	uint32_t boost_T;	// ��ѹ ���� us
	uint32_t boost_F;	// ��ѹ Ƶ�� hz
	float boost_Width;	// ��ѹ ���� ռ�ձ� 0.0~100.0
	uint16_t boost_uGroupCount;	// ��ѹ ����ÿ�鵥λ����

	// ����Ϊ����Ĳ�
	uint8_t boost_Level;	// �о� ���� 0~15
	uint32_t trig_Width;	// ������ ���� us
	float trig_T;	// ������ ���� ms
	uint16_t trig_Count;	// ������ �ϳ�ÿ�θо�Ҫ��������Ĵ��� ��
	float usense_T;	// ����ÿ�θо� ���� ms
	
	uint16_t boost_Count;	// ��ѹ ���� ÿ���ܸ���
	
} shockPluse_t;


// ΢�뼶��ʱ����
void delay_us(__IO uint32_t delay);

// ��һ��ֵ��һ����Χӳ�䵽��һ����Χ�������汾��
long longMap(long value, long in_min, long in_max, long out_min, long out_max);

// ��һ��ֵ��һ����Χӳ�䵽��һ����Χ���������汾���߾��ȣ�
double doubleMap(double value, double in_min, double in_max, double out_min, double out_max);

// ����������
void shockIOPinConfig(shockPluse_t* shockPluse_s_p);

// ��Գ�������
void shockConstConfig(shockPluse_t* shockPluse_s_p);

// ����pwmƵ�����Ĵ���ֵ
void pwmCalculatePSC_ARR(uint32_t targetFreq, uint32_t* PSC, uint32_t* ARR);

// ����pwmƵ��
uint8_t shockBoostSetFreq(shockPluse_t* shockPluse_s_p, uint32_t boostPwmFreq_HZ);

// ����pwmռ�ձ�
void shockBoostSetDuty(shockPluse_t* shockPluse_s_p, float pwmDutyCycle);

// ��̼��ܳ�ʼ��
void shockAllInit(shockPluse_t* shockPluse_s_p);

// ��ѹ����
void shockBoostVol(shockPluse_t* shockPluse_s_p);

// ������� ����
void shockTriggerAC(shockPluse_t* shockPluse_s_p, GPIO_PinState GPIO_PIN_Sta);

// ������� ֱ��
void shockTriggerDC(shockPluse_t* shockPluse_s_p, GPIO_PinState GPIO_PIN_Sta);

// �о� ��������
void shockPluseSenseSet(shockPluse_t* shockPluse_s_p);

// ������� ��λ�о�
void shockPulseSenseUnit(shockPluse_t* shockPluse_s_p);

// ������� ��������
void shockPluseFunction(shockPluse_t* shockPluse_s_p);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USER_LIB_H__ */

