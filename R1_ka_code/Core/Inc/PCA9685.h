/*
 * PCA9685.h
 *
 *  Created on: Dec 23, 2
 *  025
 *      Author: OM
 */

#ifndef PCA9685_H
#define PCA9685_H
#include "stdio.h"
#include "stm32f4xx.h"
#include <main.h>
#include <stdint.h>

typedef struct{

	I2C_HandleTypeDef *hi2c;
	uint16_t frequency;
	uint8_t address;

}PCA_t;

void InitPCA(PCA_t *pca,I2C_HandleTypeDef *hi2c, uint16_t frequency);
void RotateServo(PCA_t *pca ,uint8_t channel, uint8_t angle);

#endif /* INC_PCA9685_H_ */


