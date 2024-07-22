#include "servo.h"

#define PERIOD_TIMER_MS		10

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

static HAL_StatusTypeDef HAL_TIM_PWM_PrepareStart(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, Channel));

  /* Check the TIM channel state */
  if (TIM_CHANNEL_STATE_GET(htim, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
    return HAL_ERROR;
  }

  /* Set the TIM channel state */
  TIM_CHANNEL_STATE_SET(htim, Channel, HAL_TIM_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

  if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET)
  {
    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE(htim);
  }

  /* Return function status */
  return HAL_OK;
}

PWM_Channel::PWM_Channel(TIM_HandleTypeDef *_pwmTim, uint32_t _pwmChannel, GPIO_TypeDef *_gpioPort, uint16_t _gpioPin):
		pwmTim(_pwmTim), pwmChannel(_pwmChannel), gpioPort(_gpioPort), gpioPin(_gpioPin)
{

}

void PWM_Channel::start()
{
	HAL_TIM_PWM_PrepareStart(pwmTim, pwmChannel);
}

void PWM_Channel::stop()
{
	HAL_TIM_PWM_Stop(pwmTim, pwmChannel);
}

void PWM_Channel::setDutyCycle(float dutyPpm)
{
	uint32_t reg = uint32_t(dutyPpm*float(pwmTim->Init.Period)) % pwmTim->Init.Period;
	switch (pwmChannel) {
		case TIM_CHANNEL_1:
			pwmTim->Instance->CCR1 = reg;
			break;
		case TIM_CHANNEL_2:
			pwmTim->Instance->CCR2 = reg;
			break;
		case TIM_CHANNEL_3:
			pwmTim->Instance->CCR3 = reg;
			break;
		}
}

void PWM_Channel::setPosDirection()
{
	HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
}

void PWM_Channel::setNegDirection()
{
	HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);
}

void SPWM::start()
{
	channels[0]->setDutyCycle(0.0f);
	channels[1]->setDutyCycle(0.0f);
	channels[2]->setDutyCycle(0.0f);

	channels[0]->start();
	channels[1]->start();
	channels[2]->start();

	__HAL_TIM_ENABLE(pwmTim);
}

void SPWM::setNormVoltage(float amplitude, float angle) {
	float offset = 0.0f;
	float vec[3];
	for (uint8_t i = 0; i < 3; i++) {
		vec[i] = amplitude*std::cos((angle + offset)/180.0f*PI);
		offset += 120.0f;
	}
	for (uint8_t i = 0; i < 3; i++) {
		channels[i]->setDutyCycle(std::fabs(vec[i]));
		if (vec[i] >= 0.0f)
			channels[i]->setPosDirection();
		else
			channels[i]->setNegDirection();
	}
}


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
		vec.setNormVoltage(0.65, 360.0f*phase/100.0f);
		phase = (phase + 1) % 100;
	}
	periodCounter = (periodCounter + 1) % PERIOD_TIMER_MS;
}

void ServoStart(void)
{
	vec.start();
}

}

