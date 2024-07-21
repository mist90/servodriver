#include "servo.h"

#define PERIOD_TIMER_MS		10

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;


static PWM_Channel ch1(&htim1, TIM_CHANNEL_1, In1_1_GPIO_Port, In1_1_Pin);
static PWM_Channel ch2(&htim1, TIM_CHANNEL_2, In2_1_GPIO_Port, In2_1_Pin);
static PWM_Channel ch3(&htim1, TIM_CHANNEL_3, In3_1_GPIO_Port, In3_1_Pin);
static SPWM vec(htim1, ch1, ch2, ch3);

static uint32_t phase = 0;
static uint32_t periodCounter = 0;


extern "C" {

void PWM_TimerHandler(void)
{
	if (periodCounter == 0) {
		vec.setNormVoltage(0.65, 360.0f*phase/1000.0f);
		phase = (phase + 1) % 1000;
	}
	periodCounter = (periodCounter + 1) % PERIOD_TIMER_MS;
}

void ServoStart(void)
{
	vec.start();
}

}

