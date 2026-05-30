/*
 * PCA9685.c
 *
 *  Created on: Dec 23, 2025
 *      Author: OM
 */

#include <math.h>
#include <PCA9685.h>
#define PCA9685_ADDR (0x40 << 1)
#define MODE1 0X00
#define MODE2 0x01
#define PRE_SCALE 0xFE
#define LED0_ON_L 0x06

float Map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void InitPCA(PCA_t *pca, I2C_HandleTypeDef *hi2c, uint16_t frequency) {

	pca->hi2c = hi2c;
	pca->frequency = frequency;

	uint8_t sleep;
	HAL_I2C_Mem_Read(pca->hi2c, PCA9685_ADDR, MODE1, 1, &sleep, 1,
			HAL_MAX_DELAY);
	HAL_Delay(10);

	sleep |= (1 << 4);

	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, MODE1, 1, &sleep, 1,
			HAL_MAX_DELAY);   // put in sleep mode
	HAL_Delay(10);

	uint8_t prescaler;
	HAL_I2C_Mem_Read(pca->hi2c, PCA9685_ADDR, PRE_SCALE, 1, &prescaler, 1,
			HAL_MAX_DELAY);
	HAL_Delay(10);

	prescaler = round(25000000 / (4096 * frequency)) - 1;

	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, PRE_SCALE, 1, &prescaler, 1,
			HAL_MAX_DELAY);
	HAL_Delay(10);

	uint8_t wake;
	HAL_I2C_Mem_Read(pca->hi2c, PCA9685_ADDR, MODE1, 1, &wake, 1,
			HAL_MAX_DELAY);
	HAL_Delay(10);

	wake &= ~(1 << 4);  // disable sleep mode
	wake |= 32;       // enable auto increment register
	wake |= 128;      // enable restart

	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, MODE1, 1, &wake, 1,
			HAL_MAX_DELAY);  // enter wake mode and enable auto increment
	HAL_Delay(10);

	uint8_t mode2 = 0x04;
	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, MODE2, 1, &mode2, 1,
			HAL_MAX_DELAY);

}

void RotateServo(PCA_t *pca, uint8_t channel, uint8_t angle) {

	double time_for_high = Map(angle, 0, 200, 0.5, 2.5);
	// double time_for_high = 2.5;
	double time_for_off = 20 - time_for_high;
	double off_count_start = 4096 - (4096.0 * time_for_off) / 20.0;

	uint8_t ON_LSB = 0;
	uint8_t ON_MSB = 0;
	uint16_t OFF = off_count_start;
	uint8_t OFF_LSB = OFF & 0xFF;
	uint8_t OFF_MSB = (OFF & 0xF00) >> 8;
	uint8_t reg = LED0_ON_L + 4 * channel;

	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, reg, 1, &ON_LSB, 1,
			HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, reg + 1, 1, &ON_MSB, 1,
			HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, reg + 2, 1, &OFF_LSB, 1,
			HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(pca->hi2c, PCA9685_ADDR, reg + 3, 1, &OFF_MSB, 1,
			HAL_MAX_DELAY);

}

