/**
 * @file  tfluna_uart.c
 * @brief TF-Luna LiDAR UART driver implementation for STM32 HAL
 */

#include "tfluna_uart.h"
#include <string.h>

/* ─── Internal helpers ─────────────────────────────────────────────────── */

/**
 * Calculate TF-Luna checksum: lower 8 bits of the sum of bytes 0–7.
 */
static uint8_t calc_checksum(const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* ─── Public API ───────────────────────────────────────────────────────── */

void TFLuna_Init(TFLuna_Handle *dev, UART_HandleTypeDef *huart)
{
    dev->huart = huart;
    memset(dev->rx_buf, 0, sizeof(dev->rx_buf));
}

/* --------------------------------------------------------------------------
 * TFLuna_ReadBlocking
 *
 * Strategy:
 *   1. Read bytes one at a time until we see two consecutive 0x59 bytes.
 *   2. Read the remaining 7 bytes into the frame buffer.
 *   3. Verify checksum, then decode.
 * -------------------------------------------------------------------------- */
TFLuna_Status TFLuna_ReadBlocking(TFLuna_Handle *dev, TFLuna_Data *data)
{
    uint8_t byte;
    HAL_StatusTypeDef hal_status;

    /* --- Sync: find 0x59 0x59 header --- */
    uint8_t header_count = 0;
    while (header_count < 2)
    {
        hal_status = HAL_UART_Receive(dev->huart, &byte, 1, TFLUNA_RX_TIMEOUT_MS);
        if (hal_status == HAL_TIMEOUT) return TFLUNA_ERR_TIMEOUT;
        if (hal_status != HAL_OK)     return TFLUNA_ERR_UART;

        if (byte == TFLUNA_HEADER) {
            header_count++;
        } else {
            header_count = 0;   /* reset if broken sequence */
        }
    }

    /* Fill first two bytes manually (we already found them) */
    dev->rx_buf[0] = TFLUNA_HEADER;
    dev->rx_buf[1] = TFLUNA_HEADER;

    /* --- Read remaining 7 bytes --- */
    hal_status = HAL_UART_Receive(dev->huart,
                                  &dev->rx_buf[2],
                                  TFLUNA_FRAME_SIZE - 2,
                                  TFLUNA_RX_TIMEOUT_MS);
    if (hal_status == HAL_TIMEOUT) return TFLUNA_ERR_TIMEOUT;
    if (hal_status != HAL_OK)      return TFLUNA_ERR_UART;

    /* --- Parse and return --- */
    return TFLuna_ParseFrame(dev->rx_buf, data);
}

/* --------------------------------------------------------------------------
 * TFLuna_ParseFrame
 *
 * Frame layout (9 bytes):
 *   [0]  0x59  Header byte 1
 *   [1]  0x59  Header byte 2
 *   [2]  DistL  Distance low byte
 *   [3]  DistH  Distance high byte
 *   [4]  AmpL   Amplitude low byte
 *   [5]  AmpH   Amplitude high byte
 *   [6]  TempL  Temperature low byte
 *   [7]  TempH  Temperature high byte
 *   [8]  CKS    Checksum (sum of bytes 0–7, lower 8 bits)
 * -------------------------------------------------------------------------- */
TFLuna_Status TFLuna_ParseFrame(const uint8_t *buf, TFLuna_Data *data)
{
    /* Verify header */
    if (buf[0] != TFLUNA_HEADER || buf[1] != TFLUNA_HEADER) {
        return TFLUNA_ERR_HEADER;
    }

    /* Verify checksum */
    uint8_t expected_cks = calc_checksum(buf, TFLUNA_FRAME_SIZE - 1);
    if (buf[TFLUNA_FRAME_SIZE - 1] != expected_cks) {
        return TFLUNA_ERR_CHECKSUM;
    }

    /* Decode fields */
    data->distance_cm   = (uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8);
    data->amplitude     = (uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8);

    /* Raw temperature: divide by 8 gives °C */
    uint16_t temp_raw   = (uint16_t)(buf[6]) | ((uint16_t)(buf[7]) << 8);
    data->temperature_c = (float)temp_raw / 8.0f;

    /* Validity flag based on amplitude */
    data->valid = (data->amplitude >= TFLUNA_AMP_MIN) &&
                  (data->amplitude <  TFLUNA_AMP_MAX);

    return TFLUNA_OK;
}

/* --------------------------------------------------------------------------
 * TFLuna_SendCommand
 *
 * Command frame layout:
 *   [0]  0x5A        Frame header
 *   [1]  length      Total frame length (= 4 + payload_len)
 *   [2]  cmd_id      Command identifier
 *   [3..N] payload   Optional payload bytes
 *   [N+1] checksum   Sum of all preceding bytes, lower 8 bits
 * -------------------------------------------------------------------------- */
TFLuna_Status TFLuna_SendCommand(TFLuna_Handle *dev,
                                  uint8_t cmd_id,
                                  const uint8_t *payload,
                                  uint8_t len)
{
    /* Maximum sensible command size */
    uint8_t frame[32];
    uint8_t total_len = 4 + len;   /* header(1) + len(1) + id(1) + payload + cks(1) */

    if (total_len > sizeof(frame)) {
        return TFLUNA_ERR_UART;    /* payload too large */
    }

    frame[0] = 0x5A;
    frame[1] = total_len;
    frame[2] = cmd_id;

    for (uint8_t i = 0; i < len; i++) {
        frame[3 + i] = payload[i];
    }

    frame[total_len - 1] = calc_checksum(frame, total_len - 1);

    HAL_StatusTypeDef status = HAL_UART_Transmit(dev->huart,
                                                   frame,
                                                   total_len,
                                                   TFLUNA_RX_TIMEOUT_MS);
    return (status == HAL_OK) ? TFLUNA_OK : TFLUNA_ERR_UART;
}

TFLuna_Status TFLuna_SetFrameRate(TFLuna_Handle *dev, uint16_t fps)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)(fps & 0xFF);
    payload[1] = (uint8_t)((fps >> 8) & 0xFF);
    return TFLuna_SendCommand(dev, 0x03, payload, 2);
}

TFLuna_Status TFLuna_SoftReset(TFLuna_Handle *dev)
{
    return TFLuna_SendCommand(dev, 0x02, NULL, 0);
}

TFLuna_Status TFLuna_SaveSettings(TFLuna_Handle *dev)
{
    return TFLuna_SendCommand(dev, 0x11, NULL, 0);
}
