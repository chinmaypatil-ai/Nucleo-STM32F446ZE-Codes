/*
 * OdriveS1.c
 *
 *  Created on: Mar 31, 2026
 *      Author: pchin
 */
/*
 * OdriveS1.c
 *
 *  Created on: Mar 30, 2026
 *      Author: __pchin__
 */

#include "OdriveS1.h"
#include <stdio.h>
#include "ODriveEnums.h"
#include <stdlib.h>
#include <string.h>

static const int kMotorNumber = 0;

/* ------------------------------------------------------------------ */
/*  Internal helper: send string over UART                             */
/* ------------------------------------------------------------------ */
static void uart_send(OdriveS1 *dev, const char *str)
{
    HAL_UART_Transmit(dev->huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* ------------------------------------------------------------------ */
/*  Internal helper: read one '\n'-terminated line from UART           */
/* ------------------------------------------------------------------ */
void ODrive_readLine(OdriveS1 *dev, char *buf, uint16_t buf_len, uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint32_t start = HAL_GetTick();
    uint8_t  c;

    memset(buf, 0, buf_len);

    while (1)
    {
        /* Wait for a byte */
        while (HAL_UART_Receive(dev->huart, &c, 1, 1) != HAL_OK)
        {
            if ((HAL_GetTick() - start) >= timeout_ms)
                return;   /* timeout — return whatever we have */
        }

        if (c == '\n')
            break;

        if (idx < buf_len - 1)
            buf[idx++] = (char)c;
    }
}

/* ------------------------------------------------------------------ */
/*  clearErrors                                                         */
/* ------------------------------------------------------------------ */
void ODrive_clearErrors(OdriveS1 *dev)
{
    uart_send(dev, "sc\n");
}

/* ------------------------------------------------------------------ */
/*  closedLoopState                                                     */
/* ------------------------------------------------------------------ */
void ODrive_closedLoopState(OdriveS1 *dev)
{
    char msg[64];

    /* Wait until ODrive is reachable */
    while (ODrive_getState(dev) == AXIS_STATE_UNDEFINED)
    {
        HAL_Delay(10);
    }

    float vbus = ODrive_getParameterAsFloat(dev, "vbus_voltage");
    snprintf(msg, sizeof(msg), "DC voltage: %.4f\r\n", vbus);
    uart_send(dev, msg);

    /* Enable closed loop control */
    while (ODrive_getState(dev) != AXIS_STATE_CLOSED_LOOP_CONTROL)
    {
        ODrive_clearErrors(dev);
        ODrive_setState(dev, AXIS_STATE_CLOSED_LOOP_CONTROL);
        HAL_Delay(10);
    }

    HAL_Delay(500);
}

/* ------------------------------------------------------------------ */
/*  setPosition                                                         */
/* ------------------------------------------------------------------ */
void ODrive_setPosition(OdriveS1 *dev, float position)
{
    ODrive_setPosition_vtff(dev, position, 0.0f, 0.0f);
}

void ODrive_setPosition_vff(OdriveS1 *dev, float position, float velocity_feedforward)
{
    ODrive_setPosition_vtff(dev, position, velocity_feedforward, 0.0f);
}

void ODrive_setPosition_vtff(OdriveS1 *dev, float position,
                              float velocity_feedforward, float torque_feedforward)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "p %d %.4f %.4f %.4f\n",
             kMotorNumber, position, velocity_feedforward, torque_feedforward);
    uart_send(dev, buf);
}

/* ------------------------------------------------------------------ */
/*  setVelocity                                                         */
/* ------------------------------------------------------------------ */
void ODrive_setVelocity(OdriveS1 *dev, int motor_number, float velocity)
{
    ODrive_setVelocity_tff(dev, motor_number, velocity, 0.0f);
}

void ODrive_setVelocity_tff(OdriveS1 *dev, int motor_number,
                             float velocity, float torque_feedforward)
{
    char buf[64];
    ODrive_setMode(dev, CONTROL_MODE_VELOCITY_CONTROL);
    ODrive_setParameterInt(dev, "axis0.controller.config.input_mode",
                           INPUT_MODE_VEL_RAMP);
    snprintf(buf, sizeof(buf), "v %d %.4f %.4f\n",
             motor_number, velocity, torque_feedforward);
    uart_send(dev, buf);
}

/* ------------------------------------------------------------------ */
/*  setTorque                                                           */
/* ------------------------------------------------------------------ */
void ODrive_setTorque(OdriveS1 *dev, float torque)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "c %d %.4f\n", kMotorNumber, torque);
    uart_send(dev, buf);
}

/* ------------------------------------------------------------------ */
/*  trapezoidalMove                                                     */
/* ------------------------------------------------------------------ */
void ODrive_trapezoidalMove(OdriveS1 *dev, float position)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "t %d %.4f\n", kMotorNumber, position);
    uart_send(dev, buf);
}

/* ------------------------------------------------------------------ */
/*  goTo                                                                */
/* ------------------------------------------------------------------ */
void ODrive_goTo(OdriveS1 *dev, float position, float velocity)
{
    ODrive_setMode(dev, CONTROL_MODE_POSITION_CONTROL);
    ODrive_setParameterAsFloat(dev, "axis0.trap_traj.config.vel_limit",   velocity);
    ODrive_setParameterAsFloat(dev, "axis0.trap_traj.config.accel_limit", 30.0f);
    ODrive_setParameterAsFloat(dev, "axis0.trap_traj.config.decel_limit", 30.0f);
    ODrive_setParameterInt(dev, "axis0.controller.config.input_mode",
                           INPUT_MODE_TRAP_TRAJ);
    ODrive_trapezoidalMove(dev, position);
}

/* ------------------------------------------------------------------ */
/*  getFeedback                                                         */
/* ------------------------------------------------------------------ */
ODriveFeedback ODrive_getFeedback(OdriveS1 *dev)
{
    ODriveFeedback fb = {0.0f, 0.0f};
    char buf[64];
    char cmd[16];

    /* Flush RX — read and discard any pending bytes */
    uint8_t dummy;
    while (HAL_UART_Receive(dev->huart, &dummy, 1, 0) == HAL_OK);

    /* Request feedback */
    snprintf(cmd, sizeof(cmd), "f %d\n", kMotorNumber);
    uart_send(dev, cmd);

    /* Read response */
    ODrive_readLine(dev, buf, sizeof(buf), 10);

    /* Parse "pos vel" */
    char *space = strchr(buf, ' ');
    if (space != NULL)
    {
        *space = '\0';
        fb.pos = atof(buf);
        fb.vel = atof(space + 1);
    }

    return fb;
}

float ODrive_getPosition(OdriveS1 *dev)
{
    return ODrive_getFeedback(dev).pos;
}

float ODrive_getVelocity(OdriveS1 *dev)
{
    return ODrive_getFeedback(dev).vel;
}

/* ------------------------------------------------------------------ */
/*  Generic parameter access                                            */
/* ------------------------------------------------------------------ */
long ODrive_getParameterAsInt(OdriveS1 *dev, const char *path)
{
    char buf[64];
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "r %s\n", path);
    uart_send(dev, cmd);
    ODrive_readLine(dev, buf, sizeof(buf), 10);
    return atol(buf);
}

float ODrive_getParameterAsFloat(OdriveS1 *dev, const char *path)
{
    char buf[64];
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "r %s\n", path);
    uart_send(dev, cmd);
    ODrive_readLine(dev, buf, sizeof(buf), 10);
    return atof(buf);
}

void ODrive_setParameterStr(OdriveS1 *dev, const char *path, const char *value)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "w %s %s\n", path, value);
    uart_send(dev, buf);
}

void ODrive_setParameterInt(OdriveS1 *dev, const char *path, long value)
{
    char val_str[32];
    snprintf(val_str, sizeof(val_str), "%ld", value);
    ODrive_setParameterStr(dev, path, val_str);
}

void ODrive_setParameterAsFloat(OdriveS1 *dev, const char *path, float value)
{
    char val_str[32];
    snprintf(val_str, sizeof(val_str), "%.4f", value);
    ODrive_setParameterStr(dev, path, val_str);
}

/* ------------------------------------------------------------------ */
/*  setState / setMode                                                  */
/* ------------------------------------------------------------------ */
void ODrive_setState(OdriveS1 *dev, ODriveAxisState requested_state)
{
    ODrive_setParameterInt(dev, "axis0.requested_state", (long)requested_state);
}

void ODrive_setMode(OdriveS1 *dev, ODriveControlMode requested_mode)
{
    ODrive_setParameterInt(dev, "axis0.controller.config.control_mode",
                           (long)requested_mode);
}

/* ------------------------------------------------------------------ */
/*  getState / getProcedureResult / getError                            */
/* ------------------------------------------------------------------ */
ODriveAxisState ODrive_getState(OdriveS1 *dev)
{
    return (ODriveAxisState)ODrive_getParameterAsInt(dev, "axis0.current_state");
}

ODriveProcedureResult ODrive_getProcedureResult(OdriveS1 *dev)
{
    return (ODriveProcedureResult)ODrive_getParameterAsInt(dev, "axis0.procedure_result");
}

ODriveError ODrive_getError(OdriveS1 *dev)
{
    return (ODriveError)ODrive_getParameterAsInt(dev, "axis0.error");
}

