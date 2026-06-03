/**
 * @file    tfluna_uart.h
 * @brief   TF-Luna LiDAR UART driver for STM32 (HAL)
 *
 * Protocol: 9-byte frame @ 115200 baud (default)
 * Frame: 0x59 0x59 | DistL DistH | AmpL AmpH | TempL TempH | Checksum
 *
 * Wiring:
 *   TF-Luna TX  ->  STM32 UARTx RX
 *   TF-Luna RX  ->  STM32 UARTx TX  (optional, for config commands)
 *   TF-Luna GND ->  STM32 GND
 *   TF-Luna 5V  ->  5V supply
 */

#ifndef TFLUNA_UART_H
#define TFLUNA_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* Replace xxxx with your MCU family, e.g. f4, g0, h7 */
#include <stdint.h>
#include <stdbool.h>

/* ─── Configuration ────────────────────────────────────────────────────── */

#define TFLUNA_FRAME_SIZE       9       /**< Bytes per data frame             */
#define TFLUNA_HEADER           0x59    /**< Frame sync byte (appears twice)  */
#define TFLUNA_RX_TIMEOUT_MS    100     /**< HAL receive timeout (ms)         */

/* Amplitude thresholds (raw ADC counts) */
#define TFLUNA_AMP_MIN          100     /**< Below → weak / no return         */
#define TFLUNA_AMP_MAX          65535   /**< Above → saturation               */

/* ─── Return codes ─────────────────────────────────────────────────────── */

typedef enum {
    TFLUNA_OK           =  0,   /**< Success                                  */
    TFLUNA_ERR_TIMEOUT  = -1,   /**< UART receive timed out                   */
    TFLUNA_ERR_HEADER   = -2,   /**< Bad frame header                         */
    TFLUNA_ERR_CHECKSUM = -3,   /**< Checksum mismatch                        */
    TFLUNA_ERR_AMP      = -4,   /**< Signal amplitude out of valid range      */
    TFLUNA_ERR_UART     = -5,   /**< HAL UART error                           */
} TFLuna_Status;

/* ─── Measurement data ─────────────────────────────────────────────────── */

typedef struct {
    uint16_t distance_cm;   /**< Distance in centimetres (2 – 800 cm)         */
    uint16_t amplitude;     /**< Signal strength (100 – 65535, raw ADC)       */
    float    temperature_c; /**< Chip temperature in °C                        */
    bool     valid;         /**< true if amplitude is within usable range      */
} TFLuna_Data;

/* ─── Handle ───────────────────────────────────────────────────────────── */

typedef struct {
    UART_HandleTypeDef *huart;      /**< Pointer to HAL UART handle           */
    uint8_t rx_buf[TFLUNA_FRAME_SIZE]; /**< Raw frame buffer                  */
} TFLuna_Handle;

/* ─── Public API ───────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the TF-Luna handle.
 * @param  dev    Pointer to TFLuna_Handle to initialise.
 * @param  huart  Pointer to a HAL UART handle already configured at 115200 8N1.
 */
void TFLuna_Init(TFLuna_Handle *dev, UART_HandleTypeDef *huart);

/**
 * @brief  Read one measurement frame (blocking, with timeout).
 *
 * Searches the UART stream for a valid 0x59 0x59 header, reads the
 * remaining 7 bytes, verifies the checksum, then populates @p data.
 *
 * @param  dev   Pointer to an initialised TFLuna_Handle.
 * @param  data  Output measurement. Fields are valid only on TFLUNA_OK.
 * @retval TFLUNA_OK on success, or a TFLUNA_ERR_* code on failure.
 */
TFLuna_Status TFLuna_ReadBlocking(TFLuna_Handle *dev, TFLuna_Data *data);

/**
 * @brief  Parse a raw 9-byte buffer into a TFLuna_Data struct.
 *
 * Useful when you collect bytes yourself (e.g. DMA / interrupt mode).
 * The buffer must start at byte index 0 (0x59).
 *
 * @param  buf   9-byte frame buffer, starting with 0x59 0x59.
 * @param  data  Output measurement.
 * @retval TFLUNA_OK, TFLUNA_ERR_HEADER, or TFLUNA_ERR_CHECKSUM.
 */
TFLuna_Status TFLuna_ParseFrame(const uint8_t *buf, TFLuna_Data *data);

/**
 * @brief  Send a raw command frame to the TF-Luna.
 *
 * Constructs a command packet with the auto-calculated checksum and
 * transmits it over UART.  See the TF-Luna product manual for the full
 * command set.
 *
 * @param  dev      Pointer to an initialised TFLuna_Handle.
 * @param  cmd_id   Command byte (e.g. 0x03 for frame rate, 0x04 for baud).
 * @param  payload  Pointer to payload bytes (may be NULL if len == 0).
 * @param  len      Number of payload bytes.
 * @retval TFLUNA_OK or TFLUNA_ERR_UART.
 */
TFLuna_Status TFLuna_SendCommand(TFLuna_Handle *dev,
                                  uint8_t cmd_id,
                                  const uint8_t *payload,
                                  uint8_t len);

/**
 * @brief  Set the output frame rate (1 – 250 Hz).
 *
 * Sends command 0x03.  Default factory rate is 100 Hz.
 *
 * @param  dev  Pointer to an initialised TFLuna_Handle.
 * @param  fps  Desired frame rate in Hz.
 * @retval TFLUNA_OK or TFLUNA_ERR_UART.
 */
TFLuna_Status TFLuna_SetFrameRate(TFLuna_Handle *dev, uint16_t fps);

/**
 * @brief  Trigger a soft reset / reboot of the TF-Luna.
 * @param  dev  Pointer to an initialised TFLuna_Handle.
 * @retval TFLUNA_OK or TFLUNA_ERR_UART.
 */
TFLuna_Status TFLuna_SoftReset(TFLuna_Handle *dev);

/**
 * @brief  Save current settings to flash (persists across power cycles).
 * @param  dev  Pointer to an initialised TFLuna_Handle.
 * @retval TFLUNA_OK or TFLUNA_ERR_UART.
 */
TFLuna_Status TFLuna_SaveSettings(TFLuna_Handle *dev);

#ifdef __cplusplus
}
#endif

#endif /* TFLUNA_UART_H */
