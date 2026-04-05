/*
 * BTS7960.h
 *
 *  Created on: Mar 26, 2026
 *      Author: pchin
 */

#ifndef INC_BTS7960_H_
#define INC_BTS7960_H_



#include <stdio.h>
#include <main.h>
#include "stdint.h"
#include "stdlib.h"


/*
 * Structure Which Store PWM Value two timer address and 2 Channels of Timer
 */
typedef struct{
	 int16_t PWM_Value;
	 TIM_HandleTypeDef *htim1;
	 TIM_HandleTypeDef *htim2;
	 uint32_t Channel1;
	 uint32_t Channel2;
	 uint8_t bus1;
	 uint8_t bus2;
	 uint16_t freq;
}BTS;  // BTS it is used to define a object for  1 Motor Driver



void InitBTS(BTS *Motor , TIM_HandleTypeDef *htim1, uint32_t Channel1,uint8_t bus1,TIM_HandleTypeDef *htim2, uint32_t Channel2, uint8_t bus2,uint16_t freq);
void RotateBTS(BTS *Motor , int16_t Value);
void StopBTS(BTS *Motor);
void TestBTS(BTS *Motor);

#endif /* INC_BTS7960_H_ */
