#include "./SHOCK_MODULE/shockModule.h"

#include "tim.h"
// #include "gpio.h"


#define CPU_FREQUENCY_MHZ    72		// STM32ʱ����ƵMHZ
void delay_us(__IO uint32_t delay)	// ΢����ʱ����
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}


/**
 * ��һ��ֵ��һ����Χӳ�䵽��һ����Χ
 * 
 * @param value Ҫӳ���ֵ��
 * @param in_min ���뷶Χ����Сֵ��
 * @param in_max ���뷶Χ�����ֵ��
 * @param out_min �����Χ����Сֵ��
 * @param out_max �����Χ�����ֵ��
 * @return ӳ����ֵ��
 */
long longMap(long value, long in_min, long in_max, long out_min, long out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * ��һ��������ֵ��һ����Χӳ�䵽��һ����Χ���ṩ�߾��ȵ�ӳ��
 */
double doubleMap(double value, double in_min, double in_max, double out_min, double out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//const uint16_t boost_T = 40  /* ��ѹ ���� us */, boost_W = 750 /* ��ѹ ���� ռ�ձ� 0~1023 */, boost_G = 8 /* ��ѹ����ÿ�鵥λ���� */; //��Щֵ���ۼ���󲻱�

// ���Ĳ�
// uint16_t boost_L = 1 /* �о� ���� 0~15 */;
// uint16_t out1_W = 50 /* ������ ���� us */;
// out1_T = 3.0 /* ������ ���� ms */;
// uint16_t out1_C = 6 /* ������ �ϳ�ÿ�θо�Ҫ��������Ĵ��� �� */;
// float once_T = 100.5 /* ����ÿ�θо� ���� ms */;

// uint16_t boost_C = boost_G * boost_L /* ��ѹ���� ÿ���ܸ��� */;


// ����������	//
void shockIOPinConfig(shockPluse_t* shockPluse_s_p)
{
	//shockPluse_s_p->GPIOx_Boost_L = NULL;
	//shockPluse_s_p->GPIO_Pin_Boost_L = NULL;
	shockPluse_s_p->htim_Boost_L = &htim1;
	shockPluse_s_p->Channel_Boost_L = TIM_CHANNEL_1;
	
	shockPluse_s_p->GPIOx_Net_P = NET_P_GPIO_Port;
	shockPluse_s_p->GPIO_Pin_Net_P = NET_P_Pin;
	shockPluse_s_p->GPIOx_Net_N = NET_N_GPIO_Port;
	shockPluse_s_p->GPIO_Pin_Net_N = NET_N_Pin;
}

// ��Գ�������	//
void shockConstConfig(shockPluse_t* shockPluse_s_p)
{
	// ��Щֵ���۰�ԭ��ͼ��������󲻱�
	shockPluse_s_p->boost_T = 40;	// ��ѹ ���� us
	shockPluse_s_p->boost_F = 25000;	// ��ѹ Ƶ�� hz
	shockPluse_s_p->boost_Width = 75;	// ��ѹ ���� ռ�ձ� 0.0~100.0
	shockPluse_s_p->boost_uGroupCount = 8;	// ��ѹ ����ÿ�鵥λ����
	
	// ��������
	shockPluse_s_p->boost_Count = shockPluse_s_p->boost_uGroupCount * shockPluse_s_p->boost_Level;	// ��ѹ ���� ÿ���ܸ���
}

// ����pwmƵ�����Ĵ���ֵ	//
#define SystemCoreClock 72000000U // ʾ����72MHz��ϵͳʱ��Ƶ��
/*
void pwmCalculatePSC_ARR(uint32_t targetFreq, uint32_t* PSC, uint32_t* ARR) {	// �а�����������
	uint64_t tempPsc = 0, tempArr;
	uint64_t minDiff = UINT64_MAX;
	uint64_t targetPeriod = SystemCoreClock / targetFreq;

	// ����Ѱ����ӽ���PSC��ARR���
	for (tempPsc = 0; tempPsc < 65536; ++tempPsc) {
		tempArr = targetPeriod / (tempPsc + 1) - 1;
		if (tempArr < 65536) {
			uint64_t currentFreq = SystemCoreClock / (tempPsc + 1) / (tempArr + 1);
			uint64_t diff = targetFreq > currentFreq ? targetFreq - currentFreq : currentFreq - targetFreq;
			if (diff < minDiff) {
				minDiff = diff;
				*PSC = tempPsc;
				*ARR = tempArr;
				if (diff == 0) break; // �ҵ���ȷƥ���Ƶ��
			}
		}
	}

	if (minDiff == UINT64_MAX) {
		printf("Unable to find suitable PSC and ARR values.\n");
		
	} else {
		printf("PSC: %lu, ARR: %lu\n", (unsigned long)*PSC, (unsigned long)*ARR);
	}
}
*/

void pwmCalculatePSC_ARR(uint32_t targetFreq, uint32_t* PSC, uint32_t* ARR) {
	uint32_t psc = CPU_FREQUENCY_MHZ-1, arr = 0;
	uint64_t tempPscArrProduct = SystemCoreClock / targetFreq;

	// Ѱ�Һ��ʵ�PSC��ARRֵ
	arr = (tempPscArrProduct / (psc+1) ) - 1;

	// ����Ƿ��ҵ����ʵ�ֵ
	if (psc != 0 && arr != 0 && psc < 65536 && arr < 65536) {
		*PSC = psc;
		*ARR = arr;
		printf("PSC: %lu, ARR: %lu\n", (unsigned long)psc, (unsigned long)arr);
	} 
	else {
		*PSC = 0;
		*ARR = 0;
		printf("Unable to find suitable PSC and ARR values.\n");
	}
}

// ����pwmƵ��	//
uint8_t shockBoostSetFreq(shockPluse_t* shockPluse_s_p, uint32_t boostPwmFreq_HZ)
{
	// �����µ�PSC��ARRֵ
	uint32_t newPrescalerValue = 0; // �����µ�PWMƵ�ʼ���
	uint32_t newArrValue = 0; // �����µ�PWMƵ�ʼ���
	
	pwmCalculatePSC_ARR(boostPwmFreq_HZ, &newPrescalerValue, &newArrValue);	// ����Ƶ�ʼ��㶨ʱ���Ĵ���ֵ
	
	if (newPrescalerValue != 0 && newArrValue != 0 && newPrescalerValue < 65536 && newArrValue < 65536)
	{		
		// ֹͣPWM���
		HAL_TIM_PWM_Stop(shockPluse_s_p->htim_Boost_L, shockPluse_s_p->Channel_Boost_L);

		// ����PSC��ARR
		__HAL_TIM_SET_PRESCALER(shockPluse_s_p->htim_Boost_L, newPrescalerValue);	// 16λ��ʱ�������PSCֵΪ65535
		__HAL_TIM_SET_AUTORELOAD(shockPluse_s_p->htim_Boost_L, newArrValue);	// 16λ��ʱ�������ARRֵΪ65535

		// ʹ������Ч
		if (HAL_TIM_GenerateEvent(shockPluse_s_p->htim_Boost_L, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
			printf("Fail to Set pwmFreq");
		}

		// ��������PWM���
		HAL_TIM_PWM_Start(shockPluse_s_p->htim_Boost_L, shockPluse_s_p->Channel_Boost_L);
		
		return 0;
	}
	else {
		printf("Fail to set pwmFreq");
		return 1;
	}
}

// ����pwmռ�ձ�	//
void shockBoostSetDuty(shockPluse_t* shockPluse_s_p, float pwmDutyCycle)
{
		// ����ARRֵ��ӳ��ռ�ձȵ���ʱ���Ĵ������ó�CCRx
	__HAL_TIM_SetCompare(shockPluse_s_p->htim_Boost_L, shockPluse_s_p->Channel_Boost_L,
	 doubleMap(pwmDutyCycle, 0, 100, 0, __HAL_TIM_GET_AUTORELOAD(shockPluse_s_p->htim_Boost_L)) );    //ͨ���޸ıȽ�ֵ���ı�ռ�ձ�
	// printf("CCRx is %f\r\n", doubleMap(pwmDutyCycle, 0, 100, 0, __HAL_TIM_GET_AUTORELOAD(shockPluse_s_p->htim_Boost_L)));
}

// ��̼��ܳ�ʼ��	//
void shockAllInit(shockPluse_t* shockPluse_s_p)
{
	
	// ��ʼ�� ���ţ�����
	shockIOPinConfig(shockPluse_s_p);
	shockConstConfig(shockPluse_s_p);
	
	// ��ʼ��boost_L��pwmƵ������
	shockBoostSetFreq(shockPluse_s_p, shockPluse_s_p->boost_F); // ��������Ϊ40΢�룬��Ӧ25kHz��Ƶ��
	// ��ʼ��boost_L��pwm�����0
	shockBoostSetDuty(shockPluse_s_p, 0);
}

// ��ѹ����	//
void shockBoostVol(shockPluse_t* shockPluse_s_p)
{
	// ��������
	shockPluse_s_p->boost_Count = shockPluse_s_p->boost_uGroupCount * shockPluse_s_p->boost_Level;	// ��ѹ ���� ÿ���ܸ���
	//printf("boost_C is %d\r\n", shockPluse_s_p->boost_Count);
	
	shockBoostSetDuty(shockPluse_s_p, shockPluse_s_p->boost_Width); // ��ռ�ձȷ�������
	for (uint16_t i = 0; i < shockPluse_s_p->boost_Count; i++) {
		delay_us(shockPluse_s_p->boost_T + 1); // ѭ���ȴ�������ѹ�������
	}
	shockBoostSetDuty(shockPluse_s_p, 0); // ռ�ձ����� �ر���ѹ����
}

// ������� ����	//
void shockTriggerAC(shockPluse_t* shockPluse_s_p, GPIO_PinState GPIO_PIN_Sta)
{
	// �����ŵ�ƽ����
	HAL_GPIO_WritePin(NET_P_GPIO_Port, NET_P_Pin, GPIO_PIN_Sta);
	HAL_GPIO_WritePin(NET_N_GPIO_Port, NET_N_Pin, !GPIO_PIN_Sta);
	
	// �ȴ������������һ����
	delay_us(shockPluse_s_p->trig_Width);
	
	// ������ȫ�õ�
	HAL_GPIO_WritePin(NET_P_GPIO_Port, NET_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NET_N_GPIO_Port, NET_N_Pin, GPIO_PIN_RESET);
}

// ������� ֱ��	//
void shockTriggerDC(shockPluse_t* shockPluse_s_p, GPIO_PinState GPIO_PIN_Sta)
{
	// ֱ���������ͨ�����Ϊ�����ǵ͵�λ��
	// �����ŵ�ƽ����
	HAL_GPIO_WritePin(NET_N_GPIO_Port, NET_N_Pin, GPIO_PIN_Sta);
	// �ȴ������������һ����
	delay_us(shockPluse_s_p->trig_Width);
	// �������õ�
	HAL_GPIO_WritePin(NET_N_GPIO_Port, NET_N_Pin, GPIO_PIN_RESET);
}

// �о� ��������	//
void shockPluseSenseSet(shockPluse_t* shockPluse_s_p)
{
	// Ҫ���Ĳ�
	shockPluse_s_p->boost_Level = 1;	// �о� ���� 0~15
	shockPluse_s_p->trig_Width = 50;	// ������ ���� us
	shockPluse_s_p->trig_T = 3.0;	// ������ ���� ms
	shockPluse_s_p->trig_Count = 6;	// ������ �ϳ�ÿ�θо�Ҫ��������Ĵ��� ��
	shockPluse_s_p->usense_T = 100.5;	// ����ÿ�θо� ���� ms
	
	

}

// ������� ��λ�о�	//
void shockPulseSenseUnit(shockPluse_t* shockPluse_s_p)
{
	// �ϳ�һ�θо�
  for (uint16_t j = 0; j < shockPluse_s_p->trig_Count; j++){
		//HAL_GPIO_TogglePin(LED13_GPIO_Port,LED13_Pin);	// ָʾ��
		
    // ��ѹ����
		shockBoostVol(shockPluse_s_p);
    // ��ѹ�� �������
		shockTriggerAC(shockPluse_s_p, GPIO_PIN_SET);
		
		// ��ѹ����
		shockBoostVol(shockPluse_s_p);
		// ��ѹ�� �������
		shockTriggerAC(shockPluse_s_p, GPIO_PIN_RESET);


    //HAL_GPIO_TogglePin(LED13_GPIO_Port,LED13_Pin);	// ָʾ��

    // �ȴ���λ����������ʱ��
    HAL_Delay(shockPluse_s_p->trig_T);
  }


  // �ȴ� ��ʼ������һ�θо�
  HAL_Delay(shockPluse_s_p->usense_T);
	
}

// ������� ��������	//
void shockPluseFunction(shockPluse_t* shockPluse_s_p)
{
	
	
	
}

