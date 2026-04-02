/*
 * Cytron.c
 *
 *  Created on: Dec 16, 2025
 *      Author: Chinmay
 */
#include"Cytron.h"

void InitCytron(Cytron *Moter , TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef* DIR_PORT, uint16_t DIR_PIN, uint8_t Direction)
{
	Moter->htim = htim;
	Moter->Channel = Channel;
	Moter->GPIO_DIR_PORT = DIR_PORT;
	Moter->GPIO_DIR_PIN = DIR_PIN;
	Moter->Channel = Channel;
	Moter->Direction = Direction;
	HAL_TIM_PWM_Start(Moter->htim, Moter->Channel);

}

void RotateMoter(Cytron *Moter , int16_t Value)
{

	Moter->PWM_Value = Value;
	if(Value >= 0){
	      HAL_GPIO_WritePin(Moter->GPIO_DIR_PORT, Moter->GPIO_DIR_PIN, Moter->Direction);
	      __HAL_TIM_SET_COMPARE(Moter->htim,Moter->Channel, abs(Value));
	    }
	else if (Value < 0){
		HAL_GPIO_WritePin(Moter->GPIO_DIR_PORT, Moter->GPIO_DIR_PIN, !(Moter->Direction));
		__HAL_TIM_SET_COMPARE(Moter->htim,Moter->Channel, abs(Value));
	}
	else{
		__HAL_TIM_SET_COMPARE(Moter->htim,Moter->Channel, 0);
	}



}

void RotateForward(Cytron *Moter)
{

     HAL_GPIO_WritePin(Moter->GPIO_DIR_PORT, Moter->GPIO_DIR_PIN, 1);
	__HAL_TIM_SET_COMPARE(Moter->htim,Moter->Channel, 30);
}

void RotateReverse(Cytron *Moter)
{
    HAL_GPIO_WritePin(Moter->GPIO_DIR_PORT, Moter->GPIO_DIR_PIN, 0);
	__HAL_TIM_SET_COMPARE(Moter->htim,Moter->Channel, 30);
}

void Stop(Cytron *Moter)
{
	__HAL_TIM_SET_COMPARE(Moter->htim,Moter->Channel, 0);
}


