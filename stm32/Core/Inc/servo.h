/*
 * servo.h
 *
 *  Created on: Jul 20, 2024
 *      Author: user
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include <stdint.h>
#include <cmath>

#define PI 3.14159265

HAL_StatusTypeDef HAL_TIM_PWM_PrepareStart(TIM_HandleTypeDef *htim, uint32_t Channel)
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

class PWM_Channel
{
public:
	PWM_Channel(TIM_HandleTypeDef *_pwmTim, uint32_t _pwmChannel, GPIO_TypeDef *_gpioPort, uint16_t _gpioPin):
		pwmTim(_pwmTim), pwmChannel(_pwmChannel), gpioPort(_gpioPort), gpioPin(_gpioPin) {

	}
	void start() {
		HAL_TIM_PWM_PrepareStart(pwmTim, pwmChannel);
	}
	void stop() {
		HAL_TIM_PWM_Stop(pwmTim, pwmChannel);
	}
	void setDutyCycle(float dutyPpm) {
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
	void setPosDirection()
	{
		HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
	}
	void setNegDirection()
	{
		HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);
	}
private:
	PWM_Channel() {}
	TIM_HandleTypeDef *pwmTim;
	uint32_t pwmChannel;

	GPIO_TypeDef *gpioPort;
	uint16_t gpioPin;
};

class SPWM
{
public:
	SPWM(TIM_HandleTypeDef &_pwmTim, PWM_Channel &ch1, PWM_Channel &ch2, PWM_Channel &ch3):
		pwmTim(&_pwmTim), channels{&ch1, &ch2, &ch3} {}

	void start()
	{
		channels[0]->setDutyCycle(0.0f);
		channels[1]->setDutyCycle(0.0f);
		channels[2]->setDutyCycle(0.0f);

		channels[0]->start();
		channels[1]->start();
		channels[2]->start();

		__HAL_TIM_ENABLE(pwmTim);
	}

	void setNormVoltage(float amplitude, float angle) {
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
private:
	TIM_HandleTypeDef *pwmTim;
	PWM_Channel *const channels[3];
};



#endif /* INC_SERVO_H_ */
