/*
 * Cytron.h
 *
 *  Created on: Dec 16, 2025
 *      Author: Chinmay
 */

#ifndef INC_CYTRON_H_
#define INC_CYTRON_H_

#include <stdio.h>
#include <main.h>
#include "stdint.h"
#include "stdlib.h"

typedef struct{
	 GPIO_TypeDef* GPIO_PWM_PORT;
	 uint16_t GPIO_PWM_PIN;
	 GPIO_TypeDef* GPIO_DIR_PORT;
	 uint16_t GPIO_DIR_PIN;
	 int16_t PWM_Value;
	 uint8_t Direction;
	 TIM_HandleTypeDef *htim;
	 uint32_t Channel;
}Cytron;

void InitCytron(Cytron *Motor , TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef* GPIO_DIR_PORT, uint16_t GPIO_DIR_PIN, uint8_t Direction);
void RotateMoter(Cytron *Motor , int16_t Value);
void RotateForward(Cytron *Motor);
void RotateReverse(Cytron *Motor);
void Stop(Cytron *Motor);


#endif /* INC_CYTRON_H_ */
