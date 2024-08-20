#include "servo.h"

#define PERIOD_TIMER_MS		10

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;

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

PWM_Channel::PWM_Channel(TIM_HandleTypeDef &_pwmTim, uint32_t _pwmChannel, GPIO_TypeDef &_gpioPort, uint16_t _gpioPin):
		pwmTim(&_pwmTim), pwmChannel(_pwmChannel), gpioPort(&_gpioPort), gpioPin(_gpioPin)
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
	HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);
	isUp = true;
}

void PWM_Channel::setNegDirection()
{
	HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
	isUp = false;
}

void SPWM::start()
{
	channels[0]->setDutyCycle(0.0f);
	channels[1]->setDutyCycle(0.0f);
	channels[2]->setDutyCycle(0.0f);

	channels[0]->start();
	channels[1]->start();
	channels[2]->start();

	adcs[0]->start();
	adcs[1]->start();
	adcs[2]->start();

	__HAL_TIM_ENABLE(pwmTim);
	__HAL_TIM_ENABLE_IT(pwmTim, TIM_IT_UPDATE);
}

void SPWM::setNormVoltage(float amplitude, float angle) {
	if (initCounter != 0)
		return;
	float offset = 0.0f;
	float vec[3];
	for (uint8_t i = 0; i < 3; i++) {
		vec[i] = amplitude*std::cos((angle + offset)/180.0f*PI);
		offset += 120.0f;
	}
	for (uint8_t i = 0; i < 3; i++) {
		channels[i]->setDutyCycle(std::abs(vec[i]));
		if (vec[i] >= 0.0f)
			channels[i]->setPosDirection();
		else
			channels[i]->setNegDirection();
	}
	isCurrentMode = false;
}

void SPWM::setCurrent(float amplitude, float angle) {
	float offset = 0.0f;
	for (uint8_t i = 0; i < 3; i++) {
		currents[i] = amplitude*std::cos((angle + offset)/180.0f*PI);
		offset += 120.0f;
	}
	isCurrentMode = true;
}

void SPWM::pwmHandler(uint8_t numADC)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	/* Get current value from ADC */
	uint16_t adcValue = adcs[numADC]->value();
	/* Calculation of zero levels */
	if (initCounter != 0) {
		initCounter--;
		zeroLevels[numADC] = uint16_t((uint32_t(adcValue)*5 + uint32_t(prevZeroLevels[numADC])*95)/100);
		prevZeroLevels[numADC] = zeroLevels[numADC];
	} else if (isCurrentMode) {
		if ((__HAL_TIM_IS_TIM_COUNTING_DOWN(pwmTim) ^ (channels[numADC]->isChannelUp())) && --delays[numADC] == 0) {
			delays[numADC] = DELAY_PWM_CHANGE;
			/* Measurement of current */
			float current = float(int32_t(adcValue) - int32_t(zeroLevels[numADC]))*0.001714f;
			/* PID-controller */
			float err = currents[numADC] - current;
			currentIntegratorPI[numADC] += err*Ki*50.0f/1000000.0f;
			if (currentIntegratorPI[numADC] > PID_INT_LIMIT)
				currentIntegratorPI[numADC] = PID_INT_LIMIT;
			if (currentIntegratorPI[numADC] < -PID_INT_LIMIT)
				currentIntegratorPI[numADC] = -PID_INT_LIMIT;
			float Vout = Kp*err + currentIntegratorPI[numADC];
			/* Change duty cycle of PWM */
			float duty = std::abs(Vout);
			if (duty < 0.1)
				duty = 0.1;
			if (duty > 0.9)
				duty = 0.9;
			channels[numADC]->setDutyCycle(duty);
			if (currents[numADC] >= 0.0f)
				channels[numADC]->setPosDirection();
			else
				channels[numADC]->setNegDirection();
		}
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}


static PWM_Channel ch1(htim2, TIM_CHANNEL_1, *In1_1_GPIO_Port, In1_1_Pin);
static PWM_Channel ch2(htim2, TIM_CHANNEL_2, *In2_1_GPIO_Port, In2_1_Pin);
static PWM_Channel ch3(htim2, TIM_CHANNEL_3, *In3_1_GPIO_Port, In3_1_Pin);
static ADC_Channel adc1(hadc1);
static ADC_Channel adc2(hadc2);
static ADC_Channel adc3(hadc3);
static SPWM vec(htim2, ch1, ch2, ch3, adc1, adc2, adc3);

static uint32_t phase = 0;
static uint32_t periodCounter = 0;


extern "C" {

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc1)
		vec.pwmHandler(0);
	else if (hadc == &hadc2)
		vec.pwmHandler(1);
	else if (hadc == &hadc3)
		vec.pwmHandler(2);
}

void ServoTimerHandler(void)
{
	if (periodCounter == 0) {
		//vec.setNormVoltage(0.65, 360.0f*phase/100.0f);
		vec.setCurrent(1.0, 360.0f*phase/100.0f);
		//phase = (phase + 1) % 100;
	}
	periodCounter = (periodCounter + 1) % PERIOD_TIMER_MS;
}

void ServoStart(void)
{
	vec.start();
}

}

