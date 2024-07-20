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

class PWM_Channel
{
public:
	PWM_Channel(TIM_HandleTypeDef *_pwmTim, uint32_t _pwmChannel, GPIO_TypeDef *_gpioPort, uint16_t _gpioPin):
		pwmTim(_pwmTim), pwmChannel(_pwmChannel), gpioPort(_gpioPort), gpioPin(_gpioPin) {

	}
	void start() {
		HAL_TIM_PWM_Start(pwmTim, pwmChannel);
	}
	void stop() {
		HAL_TIM_PWM_Stop(pwmTim, pwmChannel);
	}
	void setDutyCycle(float dutyPpm) {
		switch (pwmChannel) {
			case TIM_CHANNEL_1:
				pwmTim->Instance->CCR1 = uint32_t(dutyPpm*float(pwmTim->Init.Period)) % pwmTim->Init.Period;
				break;
			case TIM_CHANNEL_2:
				pwmTim->Instance->CCR2 = uint32_t(dutyPpm*float(pwmTim->Init.Period)) % pwmTim->Init.Period;
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
	SPWM(PWM_Channel &ch1, PWM_Channel &ch2, PWM_Channel &ch3):
	channels{&ch1, &ch2, &ch3} {}

	void start()
	{
		channels[0]->setDutyCycle(0.0f);
		channels[1]->setDutyCycle(0.0f);
		channels[2]->setDutyCycle(0.0f);

		channels[0]->start();
		channels[1]->start();
		channels[2]->start();
	}

	void setNormVoltage(float amplitude, float angle) {
		float offset = 0.0f;
		for (uint8_t i = 0; i < 3; i++) {
			float vec = amplitude*std::cos((angle + offset)/180.0f*PI);
			channels[i]->setDutyCycle(std::fabs(vec));
			if (vec >= 0.0f)
				channels[i]->setPosDirection();
			else
				channels[i]->setNegDirection();
			offset += 120.0f;
		}
	}
private:
	PWM_Channel *const channels[3];
};



#endif /* INC_SERVO_H_ */
