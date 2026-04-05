/*
 * BTS7960.c
 *
 *  Created on: Mar 27, 2026
 *      Author: Chinmay
 */

#include <BTS7960.h>

/*
 * How We can Declare Object (NOTE: WORD BTS IS DECLARED IN CAPITAL LETTERS IN LIBRARY
 *                            SO IT SHOULD ENTER AS IT AS OR ERROR WILL OCCUR
 * Eg: BTS B1;
 *     BTS B2;
 */


/*****************************************************************************************
 * @fn             -   InitBTS
 *
 * @brif           -   This function Initializes the Motor Driver Pins and Timers
 *
 * @param[input]   -   Address of object that was created before
 * @param[input]   -   Timer Address for Pin LPWM
 * @param[input]   -   Timer Channel for Pin LPWM
 * @param[input]   -   APB Bus number 1 or 2 for Timer 2
 * @param[input]   -   Timer Address for Pin RPWM
 * @param[input]   -   Timer Channel for Pin RPWM
 * @param[input]   -   APB Bus number 1 or 2 for Timer 2
 * @param[input]   -   Give PWM Freq in Hz
 *
 * Example Call in main.c - (&B1, &htim1, TIM_CHANNEL_1 ,&htim3, TIM_CHANNEL_2 , 2 , 1000)
 *
 * return          -   none
 *
 * @Note           -   Timer Number and there Channel Number are CaseSentive they should be
 *                     entered as we Standard  STM32 HAL Function Example is written above
 *
 */
void InitBTS(BTS *Motor , TIM_HandleTypeDef *htim1, uint32_t Channel1,uint8_t bus1,TIM_HandleTypeDef *htim2, uint32_t Channel2,uint8_t bus2,uint16_t freq)
{
	Motor->htim1 = htim1;
	Motor->Channel1 = Channel1;
	Motor->htim2 = htim2;
	Motor->Channel2 = Channel2;
	Motor->freq = freq;
	Motor->bus1 = bus1;
	Motor->bus2 = bus2;
	uint16_t presclaer1 = 0;
	uint16_t AutoReload1 = 255;
	uint16_t presclaer2 = 0;
	uint16_t AutoReload2 = 255;

		if(Motor->bus1 == 1){
				presclaer1 = ((90000000)/(AutoReload1 * Motor->freq)-1);
			}

	    if(Motor->bus1 == 2){
				presclaer1 = ((180000000)/(AutoReload1 * Motor->freq)-1);
			}

		if(Motor->bus2 == 1){
				presclaer2 = ((90000000)/(AutoReload2 * Motor->freq)-1);
		   }

		if(Motor->bus2 == 2){
				presclaer2 = ((180000000)/(AutoReload2 * Motor->freq)-1);
		 }


	__HAL_TIM_SET_PRESCALER(Motor->htim1, presclaer1);
	__HAL_TIM_SET_AUTORELOAD(Motor->htim1,AutoReload1);
	__HAL_TIM_SET_PRESCALER(Motor->htim2, presclaer2);
	__HAL_TIM_SET_AUTORELOAD(Motor->htim2,AutoReload2);
	HAL_TIM_GenerateEvent(Motor->htim1, TIM_EVENTSOURCE_UPDATE);
	HAL_TIM_GenerateEvent(Motor->htim2, TIM_EVENTSOURCE_UPDATE);
	HAL_TIM_PWM_Start(Motor->htim1, Motor->Channel1);
	HAL_TIM_PWM_Start(Motor->htim2, Motor->Channel2);

}

/*****************************************************************************************
 * @fn             -   RotateMotor
 *
 * @brif           -   This function is used to rotate the motor in reverse or Forward Direction
 *                      depending on Sign of Value
 * @param[input]   -   Address of object that was created before
 * @param[input]   -   PWM VALUE
 *
 * Example Call in main.c - RotateMotor(&B1,40);
 *
 * return          -   none
 *
 * @Note           -   none
 *
 */
void RotateBTS(BTS *Motor , int16_t Value)
{

	Motor->PWM_Value = Value;
	if(Value >= 0){
		__HAL_TIM_SET_COMPARE(Motor->htim1,Motor->Channel1, abs(Value));
		__HAL_TIM_SET_COMPARE(Motor->htim2,Motor->Channel2, 0);
	    }
	else{
		__HAL_TIM_SET_COMPARE(Motor->htim2,Motor->Channel2, abs(Value));
		__HAL_TIM_SET_COMPARE(Motor->htim1,Motor->Channel1, 0);
	}


}

/*****************************************************************************************
 * @fn             -   StopMotor
 *
 * @brif           -   This function Stop the Motor
 *
 * @param[input]   -   Object of Motor that we have created earlier.
 *
 * Example Call in main.c - StopMotor(&B1);
 *
 * return          -   none
 *
 * @Note           -   none
 *
 */
void StopBTS(BTS *Motor){
	__HAL_TIM_SET_COMPARE(Motor->htim1,Motor->Channel1,0);
	__HAL_TIM_SET_COMPARE(Motor->htim2,Motor->Channel2,0);
}


/*****************************************************************************************
 * @fn             -   TestMotor
 *
 * @brif           -   This function is used to test Motor Driver direction if we call this
 *                     function Motor Will move Forward for 5 Seconds And Backwards for 5 seconds
 *
 * Example Call in main.c - TestMotor(&B1);
 *
 * @param[input]   -   Object of Motor that we have created earlier.
 *
 * return          -   none
 *
 * @Note           -   none
 *
 */
void TestBTS(BTS *Motor){
	__HAL_TIM_SET_COMPARE(Motor->htim1,Motor->Channel1,30);
	__HAL_TIM_SET_COMPARE(Motor->htim2,Motor->Channel2,0);
	HAL_Delay(5000);
	__HAL_TIM_SET_COMPARE(Motor->htim2,Motor->Channel2,30);
	__HAL_TIM_SET_COMPARE(Motor->htim1,Motor->Channel1,0);
	HAL_Delay(5000);
}
