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
#include"tfluna_uart.h"
#include "Encoderlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//----------------------------------------Lib objects-----------------------------------------------
PCA_t PCA;
Cytron m1, m2, m3, SM1, Belt, Turret, conveyer;
OdriveS1 M1;
encoder_instance Turr_enc;
TFLuna_Handle Kfsluna, Staffluna;
TFLuna_Data Kfsluna_Data , Staffluna_Data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// ------------------ Turret Encoder Variables----------------------------*/
int16_t encoder_velocity;
int32_t encoder_position;
uint16_t timer_counter;
float Turret_AngleNow, Turret_Error, Turret_Output;
#define COUNTS_PER_REV  28000
#define COUNTS_PER_DEG  (14000.0f / 180.0f)


//------------------------------PS4 data receive variables over UART--------------------------

int8_t L_joystick_x = 0, L_joystick_y = 0, R_joystick_x = 0, R_joystick_y = 0,
		omega = 0;
bool right;
bool down;
bool up;
bool left;
bool square;
bool cross;
bool circle;
bool triangle;
bool PS, Touchpad, Options, Share, R3, L3, R1, L1;


bool square_prev = 0;
bool cross_prev = 0;
bool circle_prev = 0;
bool triangle_prev = 0;
bool PS_prev = 0;
bool Touchpad_prev = 0;
bool Options_prev = 0;
bool Share_prev = 0;
bool R3_prev = 0;
bool L3_prev = 0;

// Global flags — set by ISR, cleared by main
volatile bool square_clicked = false;
volatile bool triangle_clicked = false;
volatile bool circle_clicked = false;
volatile bool cross_clicked = false;
volatile bool PS_clicked = false;
volatile bool R3_clicked = false;
volatile bool L3_clicked = false;
volatile bool Touchpad_clicked = false;
volatile bool Options_clicked = false;
volatile bool Share_clicked = false;

// ... add others as needed

// Previous states — only used inside ISR now
static bool square_last = false;
static bool triangle_last = false;
static bool circle_last = false;
static bool cross_last = false;
static bool PS_last = false;
static bool R3_last = false;
static bool L3_last = false;
static bool Touchpad_last = false;
static bool Options_last = false;
static bool Share_last = false;

uint8_t ModeCount = 1;

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
float angular_kp = 3.75;
float angular_ki = 0;
float angular_kd = 0.15;
float angular_p, angular_i, angular_d;
float angular_error, prev_angular_error, change_in_angular_error;
float angular_correction;
float target_heading = 0;

//chassis eqn variables

int pwm1, pwm2, pwm3;

float bot_vel, theta;


//------------------------------------------IMU variables-----------------------------------

bool imu_connected = true;
float current_heading;
float heading_normalised;
volatile bool imu = false;



typedef struct {
	bool prevState;
	bool flag;
} ToggleButton;




/*----------------------------------------- Area 1 -------------------------------------------------*/

//StaffPicking
int8_t Count_Servo_Set1 = 0,Count_Servo_Set2 = 0 , Area1_height_count = 0;
int z , y;
ToggleButton StaffHeightToggle = { 0 };
int8_t Area1_StaffHeight = 10 , Area1_PWMOutputStaff = 0;
int Area1_StaffcurrentDistance , Area1_Stafferror;


/*----------------------------------------- Area 2 -------------------------------------------------*/
int8_t Count_Kfs_height = 0 , KfspickToggleflag = 0,LeadToggleflag = 0;
int8_t Count_turret_angle = 0 , Count_Turret = 0;
int kfs_currentDistance , kfs_error ,KFS_height_set = 30 ;
int8_t kfs_output;


/*----------------------------------------- Area 3 -------------------------------------------------*/
int8_t Area3_PWMOutputStaff = 0 , Area3_StaffCount = 0 , Area3_turret_Count = 0 , Gripper_Access_Count = 0,Area3_Count_Staff_height=0;
int Area3_StaffcurrentDistance , Area3_Stafferror;
bool heightflag1 , heightflag2 , heightflag3;
uint8_t Gripper_Access_Array[5]={0,1,2,3,4};

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

void clearModeFlags(void) {
    square_clicked   = false;
    triangle_clicked = false;
    circle_clicked   = false;
    cross_clicked    = false;
    L3_clicked       = false;
    R3_clicked       = false;
    Touchpad_clicked = false;
	Share_clicked    = false;
	Options_clicked  = false;
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

bool rotate(float angle) {
	update_encoder(&Turr_enc, &htim1);
	Turret_AngleNow = normalise_angle((360.0 / COUNTS_PER_REV) * Turr_enc.position);
	Turret_Error = normalise_angle(angle - Turret_AngleNow);
    if(Turret_Error > 0) Turret_Output = 80;
    else Turret_Output = -80;
//	Turret_Output = constrain(Turret_Error * 4, -80, 80);
//	RotateCytron(&Turret, Turret_Output);

	if (fabs(Turret_Error) < 1.5) {
		return true;
	} else {
		return false;
	}
	RotateCytron(&Turret, Turret_Output);
}

void get_angle() {

	bno055_vector_t v = bno055_getVectorEuler();
	current_heading = v.x;

}
void Area1(){

/*-------------------------------------------Staff Auto Height -------------------------------------------*/
	 TFLuna_ReadBlocking(&Staffluna, &Staffluna_Data);
	 Area1_StaffcurrentDistance = Staffluna_Data.distance_cm;
	    if(Options_clicked){
	    	Options_clicked = false;
		    Area1_height_count++;
		 }

		if (Area1_height_count == 1) {
			   Area1_StaffHeight = 35;
		}
		else if(Area1_height_count == 2){
			   Area1_StaffHeight = 50;
			   Area1_height_count = 0;
		   }



/*-------------------------------------------Staff Gripper Set 1 -------------------------------------------*/
      if(triangle_clicked){
	    triangle_clicked = false;
	    Count_Servo_Set1++;
	 }
	  if(Count_Servo_Set1 == 1){
		  RotateServo(&PCA, 5, 0);
		  RotateServo(&PCA, 3, 0);

	  }
	  if(Count_Servo_Set1 == 2){
			//grip
		 RotateServo(&PCA, 5, 90);
		 RotateServo(&PCA, 3, 120);
		 Count_Servo_Set1 = 0;
	  }




/*-------------------------------------------Staff Gripper Set 2 -------------------------------------------*/
	if(circle_clicked){
	    circle_clicked = false;
	    Count_Servo_Set2++;
	}
	  if(Count_Servo_Set2 == 1){
				//grip
				RotateServo(&PCA, 4, 0);
				RotateServo(&PCA, 10, 0);
		  }

		  if(Count_Servo_Set2 == 2){
				 RotateServo(&PCA, 4, 90);
				 RotateServo(&PCA, 10, 90);

			  Count_Servo_Set2 = 0;
		   }

/*-------------------------------------------Manual Staff up/down -------------------------------------------*/
		  if (abs(R_joystick_y) > 50) {
		      // Manual override — update setpoint to current position
		      Area1_StaffHeight = Area1_StaffcurrentDistance;

		      if (R_joystick_y > 50)       RotateCytron(&SM1, 80);
		      else if (R_joystick_y < -50) RotateCytron(&SM1, -80);

		  } else {
		      // Auto height PID — only runs when joystick is idle
		      Area1_Stafferror = (float)(Area1_StaffHeight - Area1_StaffcurrentDistance);
		      if(Area1_Stafferror > 0){
		    	  Area1_PWMOutputStaff = 80;
		      }
		      else{
		    	  Area1_PWMOutputStaff = -80;
		      }
//		      Area1_PWMOutputStaff = 80; //constrain(9.0f * Area1_Stafferror, -80, 80);
		      if (abs(Area1_Stafferror) <= 2) Area1_PWMOutputStaff = 0;
		      RotateCytron(&SM1, Area1_PWMOutputStaff);
		  }


}


void Area2(){

	/*--------------------------------------Manual Conveyor Belt ---------------------------------------------------*/

	if (L1) {
			RotateCytron(&conveyer, 90);
		} else if (R1) {
			RotateCytron(&conveyer, -90);
		} else {
			RotateCytron(&conveyer, 0);
		}


	/*--------------------------------------Automated Kfs Belt ---------------------------------------------------*/
	TFLuna_ReadBlocking(&Kfsluna, &Kfsluna_Data);
	kfs_currentDistance = Kfsluna_Data.distance_cm;

	if(square_clicked){
		square_clicked = false;
		Count_Kfs_height++;
	}
		if(Count_Kfs_height == 1){
			KFS_height_set = 10;
		}

		else if(Count_Kfs_height == 2){
			KFS_height_set = 30;
		}

		else if(Count_Kfs_height == 3){
			KFS_height_set = 45;
			Count_Kfs_height = 0;
		}



if(up || down){
	KFS_height_set = kfs_currentDistance;

		if (up) {
			RotateCytron(&Belt, 120);
		}
		else if (down) {
			RotateCytron(&Belt, -120);
		}
		else {
			RotateCytron(&Belt, 0);
		}

}
 else{

	 kfs_error = KFS_height_set - kfs_currentDistance;

	 if(abs(kfs_error) <= 2){
	     kfs_output = 0;              // deadband — stop completely
	 }
	 else if(kfs_error > 0){
	     kfs_output = 120;             // too low, go up
	 }
	 else{
	     kfs_output = -120;            // too high, go down
	 }

	 RotateCytron(&Belt, kfs_output);
	}

/*--------------------------------------Turret ---------------------------------------------------*/
 if(cross_clicked){
 	  cross_clicked = false;
     Count_Turret++;
 }
    if(Count_Turret == 1){
 	   rotate(180);
    }

    if(Count_Turret == 2){
 	   rotate(120);

    }

    if(Count_Turret == 3){
 	   rotate(0);
 	   Count_Turret=0;
    }


/*--------------------------------------Kfs Gripping Servo---------------------------------------------------*/

	if(R3_clicked){
	    R3_clicked = false;
	    KfspickToggleflag++;
	}

	   if (KfspickToggleflag==1) {
           // UnGrip
		   RotateServo(&PCA, 8, 180);  // KFS Picking G2
		   RotateServo(&PCA, 1, 0);   // KFS Picking G1

	   } else if(KfspickToggleflag==2){
           // Grip
			RotateServo(&PCA, 8, 135); // KFS Picking G2
			RotateServo(&PCA, 1, 40); // KFS Picking G1
			KfspickToggleflag = 0;
	   }

 /*--------------------------------------Lead Screw ---------------------------------------------------*/

	if(Touchpad_clicked){
		Touchpad_clicked = false;
	   	    LeadToggleflag++;
	 }
	if(LeadToggleflag == 1){

	}
	if(LeadToggleflag == 2){
		LeadToggleflag = 0;
	}


}

void Area3(){
	/*--------------------------------------Manual Conveyor Belt ---------------------------------------------------*/

	if (L1) {
			RotateCytron(&conveyer, 90);
		} else if (R1) {
			RotateCytron(&conveyer, -90);
		} else {
			RotateCytron(&conveyer, 0);
		}

	/*-------------------------------------------Manual Staff up/down -------------------------------------------*/
		if (R_joystick_y > 50) {
			RotateCytron(&SM1, 80);
		} else if (R_joystick_y < -50) {
			RotateCytron(&SM1, -80);
		} else {
			Stop(&SM1);
		}

/*-------------------------------------------Manual KFS Pick -------------------------------------------*/

		if (up) {
				RotateCytron(&Belt, 120);
			}
			else if (down) {
				RotateCytron(&Belt, -120);
			}
			else {
				RotateCytron(&Belt, 0);
			}

/*-------------------------------------------Manual Turret -------------------------------------------*/

  /*-------------------------------------------Automated Staff Height for Attack -------------------------------------------*/

//		   TFLuna_ReadBlocking(&Staffluna, &Staffluna_Data);
//
//		   Area3_StaffcurrentDistance = Staffluna_Data.distance_cm;
//		if(circle_clicked){
//			circle_clicked = false;
//			Area3_StaffCount++;
//	    }
//
//				if(Area3_StaffCount == 1){
//					Area3_StaffHeight = 10;
//		            heightflag1 = true;
//				}
//
//				else if(Area3_StaffCount == 2){
//					Area3_StaffHeight = 20;
//		            heightflag2 = true;
//				}
//
//				else if(Area3_StaffCount == 3){
//					Area3_StaffHeight = 30;
//		            heightflag3 = true;
//					Area3_StaffCount = 0;
//				}
//
//			   Area3_Stafferror = (float) (Area3_StaffHeight - Area3_StaffcurrentDistance);
//			   Area3_PWMOutputStaff = constrain(15.0f * Area3_Stafferror, -90, 90);
//
//				if (abs(Area3_Stafferror) <= 2){
//					Area3_PWMOutputStaff = 0;
//                }
//				RotateCytron(&SM1, Area3_PWMOutputStaff);

/*-------------------------------------------Reset Gripper -------------------------------------------*/

		if (Options_clicked){
			Options_clicked = false;
			RotateServo(&PCA, 7, 94);			//150
			RotateServo(&PCA, 12, 94);
			RotateServo(&PCA, 2, 100);
			RotateServo(&PCA, 15, 90);
		}
/*-------------------------------------------Attacking Gripper Access -------------------------------------------*/
if (square_clicked){
	     square_clicked = false;
       Area3_Count_Staff_height++;
}



		if(Gripper_Access_Array[Area3_Count_Staff_height]==1){

            RotateServo(&PCA, 12, 155);

//			if(heightflag1){
//				RotateServo(&PCA, 12, 115);
//				heightflag1 = false;//110
//			}
//			if(heightflag2){
//				RotateServo(&PCA, 12, 115);			//110
//				heightflag2 = false;
//			}
//			if(heightflag3){
//				RotateServo(&PCA, 12, 115);			//110
//				heightflag3 = false;
//			}
		}


		if(Gripper_Access_Array[Area3_Count_Staff_height]==2){
            RotateServo(&PCA, 15, 155);

//			if(heightflag1){
//				RotateServo(&PCA, 7, 115);
//				heightflag1 = false;//110
//			}
//			if(heightflag2){
//				RotateServo(&PCA, 7, 115);			//110
//				heightflag2 = false;
//			}
//			if(heightflag3){
//				RotateServo(&PCA, 7, 115);			//110
//				heightflag3 = false;
//			}
		}


		if(Gripper_Access_Array[Area3_Count_Staff_height]==3){
            RotateServo(&PCA, 7, 155);
//			if(heightflag1){
//				RotateServo(&PCA, 7, 115);
//				heightflag1 = false;//110
//			}
//			if(heightflag2){
//				RotateServo(&PCA, 7, 115);			//110
//				heightflag2 = false;
//			}
//			if(heightflag3){
//				RotateServo(&PCA, 7, 115);			//110
//				heightflag3 = false;
//			}
		}


		if(Gripper_Access_Array[Area3_Count_Staff_height]==4){
            RotateServo(&PCA, 2, 155);
//			if(heightflag1){
//				RotateServo(&PCA, 7, 115);
//				heightflag1 = false;//110
//			}
//			if(heightflag2){
//				RotateServo(&PCA, 7, 115);			//110
//				heightflag2 = false;
//			}
//			if(heightflag3){
//				RotateServo(&PCA, 7, 115);			//110
//				heightflag3 = false;
//			}
        	if(Area3_Count_Staff_height == 4){
        		Area3_Count_Staff_height=0;
        		}
		}



  if(triangle_clicked){
	  triangle_clicked = false;

		if(Gripper_Access_Array[Gripper_Access_Count]==1){
			RotateServo(&PCA, 4, 0);
		}

		if(Gripper_Access_Array[Gripper_Access_Count]==2){
				RotateServo(&PCA, 5, 0);
		}

		if(Gripper_Access_Array[Gripper_Access_Count]==3){
				RotateServo(&PCA, 10, 10);
		}

		if(Gripper_Access_Array[Gripper_Access_Count]==4){
				RotateServo(&PCA, 3, 0);
		}

	}

 /*--------------------------------------Turret ---------------------------------------------------*/
  if(cross_clicked){
  	  cross_clicked = false;
      Count_Turret++;
  }
     if(Count_Turret == 1){
  	   rotate(180);
     }

     if(Count_Turret == 2){
  	   rotate(120);

     }

     if(Count_Turret == 3){
  	   rotate(0);
  	   Count_Turret=0;
     }


 /*--------------------------------------Lead Screw ---------------------------------------------------*/

 	if(Touchpad_clicked){
 		Touchpad_clicked = false;
 	   	    LeadToggleflag++;
 	 }
 	if(LeadToggleflag == 1){

 	}
 	if(LeadToggleflag == 2){
 		LeadToggleflag = 0;
 	}

}



void Navigation(){


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


 //find wheel velocity

			    bot_vel = sqrt(pow(L_joystick_x, 2) + pow(L_joystick_y, 2));

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

        //rotate motor
			RotateCytron(&m1, pwm1);
			RotateCytron(&m2, pwm2);
			RotateCytron(&m3, pwm3);

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
  MX_USART3_UART_Init();
  MX_FMPI2C1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	//--------------------------------------timer interrupt start------------------------
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);


    InitCytron(&m1, &htim9, TIM_CHANNEL_1, GPIOE, GPIO_PIN_6, 0, 2, 20000);
  	InitCytron(&m2, &htim8, TIM_CHANNEL_4, GPIOC, GPIO_PIN_8, 0, 2, 20000);
  	InitCytron(&m3, &htim12, TIM_CHANNEL_2, GPIOB, GPIO_PIN_14, 0, 1, 20000);
  	InitCytron(&SM1, &htim4, TIM_CHANNEL_2, GPIOF, GPIO_PIN_9, 1, 1, 20000); // DIR PF9   PWM PC6
  	InitCytron(&Turret, &htim4, TIM_CHANNEL_4, GPIOF, GPIO_PIN_8, 1, 1, 20000); // DIR PF8   PWM PA1
  	InitCytron(&Belt, &htim5, TIM_CHANNEL_3, GPIOA, GPIO_PIN_1, 1, 1, 20000); // DIR PA1
  	InitCytron(&conveyer, &htim4, TIM_CHANNEL_3, GPIOF, GPIO_PIN_7, 1, 1,20000); // DIR PF7   PWM

  	//----------------------------------------------IMU init-	------------------------------------------------
  	bno055_assignI2C(&hfmpi2c1);
  	bno055_setup();
  	bno055_setOperationModeNDOF();

  	//---------------------------------------------PCA init----------------------------------------------------
  	InitPCA(&PCA, &hi2c2, 50);
  	HAL_UART_Receive_IT(&huart2, &rxByte, 1);


	//---------------------------------- tf luna init -------------------------------------------------------------
  	TFLuna_Init(&Kfsluna, &huart5);
  	TFLuna_SetFrameRate(&Kfsluna, 250); //Set upto 0 - 250 Hz (But lower the freq more Accurate the Output)

  	TFLuna_Init(&Staffluna, &huart3);
  	TFLuna_SetFrameRate(&Staffluna, 250); //Set upto 0 - 250 Hz (But lower the freq more Accurate the Output)

//  	  HAL_Delay(100);   /* give the sensor time to apply the new setting */

  	//--------------------------------------servo init----------------------------------------------------------

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
      if (imu) {
	  		  get_angle();
	  		  imu = false;
	  		}
////		Area2();
//
//	  if(PS && !PS_prev){
//        ModeCount++;
//	if(ModeCount == 1){
//			Area1();
//			Area2();
//		Navigation();
//	  }
//	else if(ModeCount == 2){
//		Area3();
//		Navigation();
//	    ModeCount= 0;
//	}
//  }
//	  PS_prev=PS;
//
//
//	  HAL_Delay(5);
	    t_start = HAL_GetTick();
	    dt = (t_start - t_end) / 1000.0;
	    t_end = t_start;
        Navigation();
	    if(PS_clicked){
	           PS_clicked = false;
	           ModeCount++;
	           if(ModeCount > 2) ModeCount = 1;
	           clearModeFlags();
	       }
	    // Continuous mode execution
	    if(ModeCount == 1){
	        Area1();
	        Area2();
	    }
	    else if(ModeCount == 2){
	        Area3();
	    }

	       PS_prev       = PS;
	       triangle_prev = triangle;
	       circle_prev   = circle;
	       cross_prev    = cross;
	       square_prev   = square;
	       Options_prev  = Options;
	       Share_prev    = Share;
	       Touchpad_prev = Touchpad;
	       R3_prev       = R3;
	       L3_prev       = L3;

	  HAL_Delay(5);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
            	// After all the bit unpacking in the ISR:

            	// Detect rising edges and set click flags
            	if(square && !square_last)     square_clicked = true;
            	if(triangle && !triangle_last) triangle_clicked = true;
            	if(circle && !circle_last)     circle_clicked = true;
            	if(cross && !cross_last)       cross_clicked = true;
            	if(PS && !PS_last)             PS_clicked = true;
            	if(R3 && !R3_last)             R3_clicked = true;
            	if(L3 && !L3_last)             L3_clicked = true;
            	if(Touchpad && !Touchpad_last)       Touchpad_clicked = true;
            	if(Options && !Options_last)        Options_clicked = true;
            	if(Share && !Share_last)          Share_clicked = true;


            	// Update last states
            	square_last   = square;
            	triangle_last = triangle;
            	circle_last   = circle;
            	cross_last    = cross;
            	PS_last       = PS;
            	R3_last       = R3;
            	L3_last       = L3;
            	Touchpad_last = Touchpad;
            	Options_last  = Options;
            	Share_last    = Share;

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
	}

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

#ifdef  USE_FULL_ASSERT
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
