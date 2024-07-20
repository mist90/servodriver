#include "servo.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim11;
extern UART_HandleTypeDef huart2;


static PWM_Channel ch1(&htim9, TIM_CHANNEL_1, In1_1_GPIO_Port, In1_1_Pin);
static PWM_Channel ch2(&htim9, TIM_CHANNEL_2, In2_1_GPIO_Port, In2_1_Pin);
static PWM_Channel ch3(&htim11, TIM_CHANNEL_1, In3_1_GPIO_Port, In3_1_Pin);
static SPWM vec(ch1, ch2, ch3);

static uint32_t counter = 0;

extern "C" {

void PWM_TimerHandler(void)
{
	vec.setNormVoltage(0.6, 360.0f*counter/1000.0f);
	counter = (counter + 1) % 1000;
}

void ServoStart(void)
{
	vec.start();
}

}

