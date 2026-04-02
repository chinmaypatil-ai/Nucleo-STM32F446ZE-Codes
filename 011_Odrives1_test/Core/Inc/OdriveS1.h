/*
 * OdriveS1.h
 *
 *  Created on: Mar 30, 2026
 *      Author: pchin
 */

#ifndef INC_ODRIVES1_H_
#define INC_ODRIVES1_H_

#include "ODriveEnums.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct ODriveFeedback{
    float pos;
    float vel;
}ODriveFeedback;

typedef struct{
	UART_HandleTypeDef *huart;
}OdriveS1;


/* ------------------------------------------------------------------ */
/*  Function declarations                                               */
/* ------------------------------------------------------------------ */
void ODrive_setState            (OdriveS1 *dev, ODriveAxisState requested_state);
void ODrive_setMode             (OdriveS1 *dev, ODriveControlMode requested_mode);
void ODrive_setParameterAsFloat (OdriveS1 *dev, const char *path, float value);
void ODrive_setParameterStr     (OdriveS1 *dev, const char *path, const char *value);
void ODrive_setParameterInt     (OdriveS1 *dev, const char *path, long value);void ODrive_trapezoidalMove     (OdriveS1 *dev, float position);
void  ODrive_clearErrors(OdriveS1 *dev);
/** Custom: request Closed-Loop state */
void ODrive_closedLoopState    (OdriveS1 *dev);

/** Send a position setpoint */
void ODrive_setPosition        (OdriveS1 *dev, float position);

/** Send a position setpoint with velocity feedforward */
 void ODrive_setPosition_vff    (OdriveS1 *dev, float position,
                                            float velocity_feedforward);

/** Send a position setpoint with velocity and torque feedforward */
 void ODrive_setPosition_vtff   (OdriveS1 *dev, float position, float velocity_feedforward,float torque_feedforward);

/** Send a velocity setpoint */
 void ODrive_setVelocity        (OdriveS1 *dev, int motor_number,float velocity);

/** Send a velocity setpoint with torque feedforward */
  void ODrive_setVelocity_tff   (OdriveS1 *dev, int motor_number, float velocity, float torque_feedforward);

/** Send a torque setpoint */
  void ODrive_setTorque          (OdriveS1 *dev, float torque);

/** Trapezoidal-trajectory move to position */
  void ODrive_trapezoidalMove    (OdriveS1 *dev, float position);

/**
 * Custom: switch to position control, set velocity limit, then go to position.
 */
   void ODrive_goTo  (OdriveS1 *dev, float position, float velocity);

/**
 * Request latest position and velocity estimates.
 * Returns pos=0.0 and vel=0.0 on communication error.
 */
   ODriveFeedback   ODrive_getFeedback  (OdriveS1 *dev);

/** Convenience: latest position estimate (0.0 on error) */
   float ODrive_getPosition  (OdriveS1 *dev);

/** Convenience: latest velocity estimate (0.0 on error) */
   float ODrive_getVelocity  (OdriveS1 *dev);

/**
 * Generic parameter read – result written into buf (max buf_len bytes).
 * Returns number of bytes written (excluding NUL), or -1 on error.
 */
   char *  ODrive_getParameterAsString (OdriveS1 *dev, const char *path, char *buf, uint16_t buf_len);

/** Generic parameter read as integer */
   long ODrive_getParameterAsInt    (OdriveS1 *dev, const char *path);

/** Generic parameter read as float */
   float ODrive_getParameterAsFloat  (OdriveS1 *dev, const char *path);

/** Generic parameter write (string value) */
   void ODrive_setParameterStr   (OdriveS1 *dev, const char *path, const char *value);

/** Generic parameter write (integer value) */
  void ODrive_setParameterInt   (OdriveS1 *dev, const char *path, long value);

/** Tell the ODrive to change its axis state */
  void ODrive_setState         (OdriveS1 *dev,ODriveAxisState requested_state);

/** Custom: tell the ODrive to change its control mode */
   void ODrive_setMode          (OdriveS1 *dev,ODriveControlMode requested_mode);

/**
 * Request current axis state.
 * Returns AXIS_STATE_UNDEFINED on communication error.
 */
   ODriveAxisState        ODrive_getState           (OdriveS1 *dev);
   ODriveProcedureResult  ODrive_getProcedureResult (OdriveS1 *dev);
   ODriveError            ODrive_getError           (OdriveS1 *dev);

/* ------------------------------------------------------------------ */
/*  Internal / private helper (declare here, use only in .c file)      */
/* ------------------------------------------------------------------ */

/**
 * Read one '\n'-terminated line from the UART into buf.
 * Returns number of bytes read (excluding '\n'), or -1 on timeout.
 * timeout_ms: how long to wait (default 10 ms in the original C++ code).
 */

   void             ODrive_readLine           (OdriveS1 *dev, char *buf, uint16_t buf_len, uint32_t timeout_ms);
   #define          ODrive_readLine_default(dev, buf, len) ODrive_readLine(dev, buf, len, 10)

#endif /* INC_ODRIVES1_H_ */
