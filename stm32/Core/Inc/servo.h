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
	PWM_Channel(TIM_HandleTypeDef *_pwmTim, uint32_t _pwmChannel, GPIO_TypeDef *_gpioPort, uint16_t _gpioPin);
	void start();
	void stop();
	void setDutyCycle(float dutyPpm);
	void setPosDirection();
	void setNegDirection();
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

	void start();
	void setNormVoltage(float amplitude, float angle) ;
private:
	TIM_HandleTypeDef *pwmTim;
	PWM_Channel *const channels[3];
};



#endif /* INC_SERVO_H_ */
