/*
 * servo.h
 *
 *  Created on: Jul 20, 2024
 *      Author: user
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include <cstdint>
#include <cmath>

#define PI 					3.14159265f
#define Kp 					0.1f
#define Ki 					5.0f
#define PID_INT_LIMIT		50.0f

class PWM_Channel
{
public:
	PWM_Channel(TIM_HandleTypeDef &_pwmTim, uint32_t _pwmChannel, GPIO_TypeDef &_gpioPort, uint16_t _gpioPin);
	void start();
	void stop();
	void setDutyCycle(float dutyPpm);
	void setPosDirection();
	void setNegDirection();
	bool isChannelUp() {
		return isUp;
	}
private:
	PWM_Channel(): pwmTim(NULL), gpioPort(NULL) {}
	TIM_HandleTypeDef *const pwmTim;
	uint32_t pwmChannel;

	GPIO_TypeDef *const gpioPort;
	uint16_t gpioPin;

	bool isUp = false;
};

class ADC_Channel
{
public:
	ADC_Channel(ADC_HandleTypeDef &_adc): adc(&_adc) {}
	void start() {
		HAL_ADC_Start(adc);
	}
	uint16_t value() {
		HAL_ADC_PollForConversion(adc, 1);
		return HAL_ADC_GetValue(adc);
	}
private:
	ADC_HandleTypeDef *const adc;
};

class SPWM
{
public:
	SPWM(TIM_HandleTypeDef &_pwmTim, PWM_Channel &ch1, PWM_Channel &ch2, PWM_Channel &ch3,
			ADC_Channel &adc1, ADC_Channel &adc2, ADC_Channel &adc3):
		pwmTim(&_pwmTim), channels{&ch1, &ch2, &ch3}, adcs{&adc1, &adc2, &adc3} {}

	void start();
	void setNormVoltage(float amplitude, float angle);
	void setCurrent(float amplitude, float angle);
	void pwmHandler();
private:
	TIM_HandleTypeDef *const pwmTim;
	PWM_Channel *const channels[3];
	ADC_Channel *const adcs[3];

	float currents[3] = {0.0f};
	uint16_t zeroLevels[3] = {0};
	uint16_t prevZeroLevels[3] = {0};
	uint8_t curChannel = 0;
	bool isCurrentMode = false;
	uint32_t initCounter = 10000;

	float currentIntegratorPI[3] = {0};
};



#endif /* INC_SERVO_H_ */
