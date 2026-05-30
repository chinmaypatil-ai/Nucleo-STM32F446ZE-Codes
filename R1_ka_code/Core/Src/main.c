/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<stdbool.h>
#include<stdint.h>
#include<string.h>
#include<math.h>

#include<Cytron.h>
#include"BNO055_STM32.h"
#include"OdriveS1.h"
#include"PCA9685.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */


//----------------------------------------Lib objects-----------------------------------------------
PCA_t PCA;
Cytron m1, m2, m3,SM1, Belt, Turret, conveyer;
OdriveS1 M1;

//------------------------------PS4 data receive variables over UART--------------------------

int8_t L_joystick_x = 0, L_joystick_y = 0, R_joystick_x = 0, R_joystick_y = 0, omega = 0;
bool right;
bool down;
bool up;
bool left;
bool square;
bool cross;
bool circle;
bool triangle;
bool PS, Touchpad, Options, Share, R3, L3, R1, L1;

//#define data_size 6
//uint8_t rxByte;
//int8_t data[data_size];
uint8_t rxByte;

int8_t data[6];

uint8_t receiving = 0;

uint8_t index1 = 0;

uint8_t length = 0;


//----------------------------------Navigation variables--------------------------------------------



#define radius       1
#define length       1
#define MAX_PWM      140
#define DEADZONE     30

bool flag = false;
bool is_omega = false;
bool pid = true;

unsigned long t_start, t_end;
float dt;

//angular PID
float angular_kp = 3.00;
float angular_ki = 1.6;
float angular_kd = 0.15;
float angular_p, angular_i, angular_d;
float angular_error, prev_angular_error, change_in_angular_error;
float angular_correction;
float target_heading = 0;

//chassis eqn variables

int pwm1, pwm2, pwm3;

float bot_vel, theta;


//------------------------------------------IMU variables-----------------------------------

float current_heading;
float heading_normalised;
volatile bool imu = false;


// ------------------------------------KFS placing variables------------------------------------


typedef enum {
	HOME, PLACE, REACH,
} ODrivePosition;

typedef struct {
	bool prevState;
} ODriveButton_t;

ODriveButton_t ODrive = { 0 };
ODriveButton_t ODrivePlace = { 0 };

ODrivePosition ODrivePos = HOME;

Cytron Turret, kfs_y_motion;


	typedef enum {
		OPEN, KEEP, GRAB,
	} servoPosition;

	typedef struct {
		bool prevState;
	} servoButton_t;

	servoButton_t servo = { 0 };
	servoButton_t servoPlace = { 0 };

	servoPosition servoPos = OPEN;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float radians(float angle) {
	return angle * 2 * 3.14 / 360.0;
}

float degrees(float angle) {
	return angle * 360.0 / (2 * 3.14);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float num, float min, float max) {

	if (num > max) {
		return max;
	}

	else if (num < min) {
		return min;
	}

	else {
		return num;
	}
}

float normalise_angle(float angle) {
	if (angle > 180.0) {
		angle -= 360.0;
	}
	if (angle < -180.0) {
		angle += 360.0;
	}
	return angle;
}


void process_data()
{
//	HAL_UART_Receive(&huart2, &rxByte, 1, HAL_MAX_DELAY);
//
//			  if(rxByte == 0xAA)
//			  {
//			      HAL_UART_Receive(&huart2, (uint8_t*)data, sizeof(data), HAL_MAX_DELAY);

	            	uint8_t TempData1 = data[0];
	            	uint8_t TempData2 = data[1];

	            		up = ((TempData1 & (1 << 0)) ? 1 : 0);
	            		right = ((TempData1 & (1 << 1)) ? 1 : 0);
	            		down = ((TempData1 & (1 << 2)) ? 1 : 0);
	            		left = ((TempData1 & (1 << 3)) ? 1 : 0);
	            		triangle = ((TempData1 & (1 << 4)) ? 1 : 0);
	            		circle = ((TempData1 & (1 << 5)) ? 1 : 0);
	            		cross = ((TempData1 & (1 << 6)) ? 1 : 0);
	            		square = ((TempData1 & (1 << 7)) ? 1 : 0);

	            		L1 = ((TempData2 & (1 << 0)) ? 1 : 0);
	            		R1 = ((TempData2 & (1 << 1)) ? 1 : 0);
	            		L3 = ((TempData2 & (1 << 2)) ? 1 : 0);
	            		R3 = ((TempData2 & (1 << 3)) ? 1 : 0);
	            		Share = ((TempData2 & (1 << 4)) ? 1 : 0);
	            		Options = ((TempData2 & (1 << 5)) ? 1 : 0);
	            		Touchpad = ((TempData2 & (1 << 6)) ? 1 : 0);
	            		PS = ((TempData2 & (1 << 7)) ? 1 : 0);

	            		omega = -data[5];
	            		R_joystick_y = data[4];
//            	            		R_joystick_x = data[4];
	            		L_joystick_x = data[2];
	            		L_joystick_y = data[3];

	            		if (abs(L_joystick_x) < DEADZONE)
	            			L_joystick_x = 0;

	            		if (abs(L_joystick_y) < DEADZONE)
	            			L_joystick_y = 0;
	            		if (abs(omega) < 5)
	            			omega = 0;


//	            }


}




void find_wheel_vel() {

	bot_vel = sqrt(pow(L_joystick_x, 2) + pow(L_joystick_y, 2));
//	theta = atan2(L_joystick_y, L_joystick_x) + radians(target_heading);
	theta = atan2(L_joystick_y, L_joystick_x);

	pwm1 = constrain(
			((bot_vel * cos(theta) - (length * omega)) / radius)
					+ angular_correction, -MAX_PWM, MAX_PWM) * 1.3;
	pwm2 = constrain(
			((sqrt(3) * bot_vel * sin(theta) - bot_vel * cos(theta)
					- 2 * length * omega) / (2 * radius)) + angular_correction,
			-MAX_PWM, MAX_PWM) * 1.3;
	pwm3 = constrain(
			(-1
					* ((bot_vel * cos(theta)
							+ sqrt(3) * bot_vel * sin(theta)
							+ 2 * length * omega) / (2 * radius)))
					+ angular_correction, -MAX_PWM, MAX_PWM) * 1.3;

}

void rotate_motor(int pwm1, int pwm2, int pwm3) {

	RotateCytron(&m1, pwm1);
	RotateCytron(&m2, pwm2);
	RotateCytron(&m3, pwm3);

}

void get_angle() {

	bno055_vector_t v = bno055_getVectorEuler();
	current_heading = v.x;

}

void PID2() {

	angular_error = normalise_angle(target_heading - current_heading);
	change_in_angular_error = angular_error - prev_angular_error;
	prev_angular_error = angular_error;

	angular_p = angular_error * angular_kp;

	angular_i = angular_i + (angular_error * (float) dt * angular_ki);
	angular_d = (change_in_angular_error / (float) dt) * angular_kd;

	if (fabs(angular_error) > 9.5 || fabs(angular_error) < 1) {
		angular_i = 0;
	}

	angular_correction = angular_p + angular_i + angular_d;

}

void STAFF_Motor() {
	if (R_joystick_y > 50) {
		RotateCytron(&SM1, 80);
	} else if (R_joystick_y < -50) {
		RotateCytron(&SM1, -80);
	} else {
		Stop(&SM1);
	}

}

void TurretMotor() {
	if (right) {
		RotateCytron(&Turret, 70);
	} else if (left) {
		RotateCytron(&Turret, -70);
	} else {
		Stop(&Turret);
	}
}

void BeltMotor() {
	if (up) {
		RotateCytron(&Belt, 90);
	} else if (down) {
		RotateCytron(&Belt, -90);
	} else {
		Stop(&Belt);
	}
}

void ConveyerMotor() {

	if (L1) {
		RotateCytron(&conveyer, 120);
	} else if (R1) {
		RotateCytron(&conveyer, -120);
	} else {
		RotateCytron(&conveyer, 0);
	}

}
void kfs_picking_placing(){

	static int lastServoPos = -1;

	    if (circle && !servo.prevState) {
	        servo.prevState = true;

	        if (servoPos == GRAB)
	            servoPos = OPEN;
	        else
	            servoPos = GRAB;
	    }
	    else if (!circle) {
	        servo.prevState = false;
	    }

		if (cross && !servoPlace.prevState) {
			servoPlace.prevState = true;

			if (servoPos == KEEP) {
				servoPos = OPEN;
			} else {
				servoPos = KEEP;
			}

		} else if (!cross) {
			servoPlace.prevState = false;
		}

	    // Send I2C commands only if position changed
	    if (lastServoPos != servoPos)
	    {
	        lastServoPos = servoPos;

	        switch (servoPos)
	        {
	        case OPEN:
	            RotateServo(&PCA, 8, 110);
	            RotateServo(&PCA, 1, 80);
	            break;

	        case KEEP:
	            RotateServo(&PCA, 8, 75);
	            RotateServo(&PCA, 1, 150);
	            break;

	        case GRAB:
	            RotateServo(&PCA, 8, 40);
	            RotateServo(&PCA, 1, 180);
	            break;
	        }
	    }

}

typedef struct {
	bool prevState;
	bool flag;
} ToggleButton;

ToggleButton StaffGripServo = { 0 };
ToggleButton StaffAttackServo = { 0 };
void STAFF_Servo() {
	// GRIP SERVO
	if (square && !StaffGripServo.prevState) {
		StaffGripServo.prevState = true;
		StaffGripServo.flag = !StaffGripServo.flag;

		if (StaffGripServo.flag) {

			RotateServo(&PCA, 5, 0);
			RotateServo(&PCA, 3, 0);
			RotateServo(&PCA, 4, 0);
			RotateServo(&PCA, 10, 10);
		} else {

			RotateServo(&PCA, 5, 90);
			RotateServo(&PCA, 3, 120);
			RotateServo(&PCA, 4, 90);
			RotateServo(&PCA, 10, 90);
		}
	} else if (!square) {
		StaffGripServo.prevState = false;
	}

	// ATTACK SERVO
	if (triangle && !StaffAttackServo.prevState) {
		StaffAttackServo.prevState = true;
		StaffAttackServo.flag = !StaffAttackServo.flag;

		if (StaffAttackServo.flag) {
			//			RotateServo(&PCA, ATTACK_SERVO_1, ATTACK_SERVO_1_ATTACK_ANGLE);
			RotateServo(&PCA, 7, 155);			//110
			RotateServo(&PCA, 12, 155);
				RotateServo(&PCA, 2, 155);
				RotateServo(&PCA, 15, 155);
		} else {
			//			RotateServo(&PCA, ATTACK_SERVO_1, ATTACK_SERVO_1_NORMAL_ANGLE);
			RotateServo(&PCA, 7, 94);			//150
			RotateServo(&PCA, 12, 94);
			RotateServo(&PCA, 2, 100);
			RotateServo(&PCA, 15, 90);
		}
	} else if (!triangle) {
		StaffAttackServo.prevState = false;
	}

}



void M1_ODrive() {
	if (Touchpad && !ODrive.prevState) {
		ODrive.prevState = true;

		if (ODrivePos == REACH) {
			ODrivePos = HOME;
		} else {
			ODrivePos = REACH;
		}

	} else if (!Touchpad) {
		ODrive.prevState = false;
	}

	if (Share && !ODrivePlace.prevState) {
		ODrivePlace.prevState = true;

		if (ODrivePos == PLACE) {
			ODrivePos = HOME;
		} else {
			ODrivePos = PLACE;
		}

	} else if (!Share) {
		ODrivePlace.prevState = false;
	}

	switch (ODrivePos) {
	case HOME:
		ODrive_goTo(&M1, 0.0f, 15.0f);
		break;
	case PLACE:
		ODrive_goTo(&M1, 6.0f, 15.0f);
		break;
	case REACH:
		ODrive_goTo(&M1, 22.0f, 15.0f);
		break;
	default:
		ODrive_goTo(&M1, 0.0f, 00);
		break;
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_FMPI2C1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //--------------------------------------timer interrupt start------------------------

  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);


  //------------------------------------Odrive init------------------------------------

  M1.huart = &huart6;
//  ODrive_clearErrors(&M1);

  //-------------------------------------motor Init------------------------------------------------------
   InitCytron(&m1, &htim9, TIM_CHANNEL_1, GPIOE, GPIO_PIN_6, 1, 2, 20000);
   InitCytron(&m2, &htim8, TIM_CHANNEL_4, GPIOC, GPIO_PIN_8, 1, 2, 20000);
   InitCytron(&m3, &htim12, TIM_CHANNEL_2, GPIOB, GPIO_PIN_14, 1, 1, 20000);
   InitCytron(&SM1, &htim4, TIM_CHANNEL_2, GPIOF, GPIO_PIN_9, 0,  1,20000); // DIR PF9   PWM PC6
   InitCytron(&Turret, &htim4, TIM_CHANNEL_4, GPIOF, GPIO_PIN_8,0,1,20000); // DIR PF8   PWM PA1
   InitCytron(&Belt, &htim5, TIM_CHANNEL_3, GPIOA, GPIO_PIN_1, 0,1,20000); // DIR PA1   PWM PF8
   InitCytron(&conveyer, &htim4, TIM_CHANNEL_3, GPIOF, GPIO_PIN_7, 1,1,20000); // DIR PF7   PWM

   //----------------------------------------------IMU init-------------------------------------------------
   bno055_assignI2C(&hfmpi2c1);
   bno055_setup();
   bno055_setOperationModeNDOF();

   //---------------------------------------------PCA init----------------------------------------------------
   InitPCA(&PCA, &hi2c2, 50);

   HAL_UART_Receive_IT(&huart2, &rxByte, 1);

	RotateServo(&PCA, 7, 180);			//110
	RotateServo(&PCA, 12, 180);
	RotateServo(&PCA, 2, 180);
	RotateServo(&PCA, 15, 180);

	HAL_Delay(3000);

	RotateServo(&PCA, 7, 115);			//110
	RotateServo(&PCA, 12, 115);
	RotateServo(&PCA, 2, 121);
	RotateServo(&PCA, 15, 115);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	            t_start = HAL_GetTick();
	  			dt = (t_start - t_end) / 1000.0;
	  			t_end = t_start;

	  			process_data();
                kfs_picking_placing();
                STAFF_Servo();
                M1_ODrive();
                STAFF_Motor();
                TurretMotor();
                BeltMotor();
                ConveyerMotor();

	  				if (imu) {
	  					get_angle();
	  					imu = false;
	  				}

	  				if (omega) {
	  					   pid = false;
	  						flag = true;
	  						  		}

	  					if (!omega && flag) {
	  						 flag = false;
	  						 pid = true;
	  						 target_heading = current_heading;
	  						  				}

	  				if (pid) {
	  					PID2();
	  				}

	  				find_wheel_vel();
	  				rotate_motor(pwm1, pwm2, pwm3);

	  			HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0xC0000E12;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{

    if(huart->Instance == USART2)

    {

        // START BYTE DETECTED

        if(rxByte == 0xAA)

        {

            receiving = 1;

            index1 = 0;

        }



        else if(receiving)

        {

            data[index1++] = rxByte;


            if(index1 >= 6)

            {

                receiving = 0;


            }

        }



        HAL_UART_Receive_IT(&huart2, &rxByte, 1);

    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM7) {
		imu = true;
//		  M1_ODrive();
	}

//	if(htim->Instance == TIM6){
//        M1_ODrive();

//	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
