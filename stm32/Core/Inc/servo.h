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

#define PI 					3.14159265f
#define Kp 					0.3f
#define Ki 					5.0f
#define PID_INT_LIMIT		500.0f
#define PID_OUT_LIMIT		0.55f

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
	SPWM(TIM_HandleTypeDef &_pwmTim, PWM_Channel &ch1, PWM_Channel &ch2, PWM_Channel &ch3, ADC_HandleTypeDef *_adc):
		pwmTim(&_pwmTim), channels{&ch1, &ch2, &ch3}, adc(_adc) {}

	void start();
	void setNormVoltage(float amplitude, float angle);
	void setCurrent(float amplitude, float angle);
	void pwmHandler();
private:
	TIM_HandleTypeDef *pwmTim;
	PWM_Channel *const channels[3];
	ADC_HandleTypeDef *adc;

	float currents[3] = {0.0f};
	uint16_t zeroLevels[3] = {0};
	uint16_t prevZeroLevels[3] = {0};
	uint8_t curChannel = 0;
	bool isCurrentMode = false;
	uint32_t initCounter = 10000;

	float currentIntegratorPI[3] = {0};
};



#endif /* INC_SERVO_H_ */
