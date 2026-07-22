/**
 ******************************************************************************
 * @file    k33_modbus.h
 * @brief   Generic Modbus-RTU transport used by the K33 sensor library.
 *          Transport-agnostic w.r.t. WHICH UART: bound at runtime via
 *          K33_Modbus_Init(). Comm strategy (polling / IT / DMA+IDLE) is
 *          selected at compile time in k33_config.h.
 ******************************************************************************
 */
#ifndef K33_MODBUS_H
#define K33_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "k33_config.h"

typedef enum
{
    K33_OK = 0,
    K33_ERR_TIMEOUT,        /* no response within K33_RESPONSE_TIMEOUT_MS */
    K33_ERR_CRC,             /* CRC mismatch on received frame */
    K33_ERR_EXCEPTION,       /* sensor replied with a Modbus exception frame */
    K33_ERR_LEN,              /* received length didn't match what was expected */
    K33_ERR_BUSY,             /* bus mutex could not be taken */
    K33_ERR_PARAM,            /* bad argument (e.g. frame too long for K33_MAX_FRAME_LEN) */
    K33_ERR_UART,             /* HAL_UART_* call itself failed */
    K33_ERR_NOT_INIT,
} K33_Status_t;

/**
 * @brief  One-time init. Call from your project (e.g. after MX_USARTx_Init())
 *         passing whichever UART_HandleTypeDef you wired to the sensor.
 *         Creates the internal FreeRTOS mutex + semaphore.
 * @note   Must be called before any other K33_* function, and only once.
 */
K33_Status_t K33_Modbus_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Send a Modbus RTU request and block (task context only - never
 *         call from an ISR) until the response is captured or times out.
 *
 * @param  req            Full request PDU: address + function + data (NO CRC -
 *                         this function appends CRC for you).
 * @param  req_len         Length of req in bytes.
 * @param  resp             Buffer to receive the raw response frame (incl. CRC).
 * @param  resp_buf_size     Capacity of resp.
 * @param  exp_resp_len       Exact expected response length in bytes, used to size
 *                         the DMA/IT receive window in K33_COMM_IT / K33_COMM_DMA_IDLE
 *                         modes. Pass 0 in K33_COMM_DMA_IDLE mode if unknown - IDLE
 *                         line detection will still find the frame boundary.
 * @param  actual_len        Out: number of bytes actually received.
 */
K33_Status_t K33_Modbus_Transact(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t resp_buf_size,
                                  uint16_t exp_resp_len, uint16_t *actual_len);

/* CRC16 (Modbus polynomial), exposed for unit testing / diagnostics. */
uint16_t K33_CRC16(const uint8_t *buf, uint16_t len);

/* ---- Wire these into your HAL callbacks (stm32h7xx_it.c or main.c) ----
 * Only the callback(s) relevant to the K33_COMM_MODE you picked actually
 * need to be reachable; calling the others is harmless (they just won't
 * ever fire for a UART instance not driven by this library). If you use
 * more than one UART in the project, check huart->Instance before/parallel
 * to forwarding into these, or forward unconditionally - the library
 * checks internally that the event is for the K33's own handle. */
void K33_Modbus_RxCpltCallback(UART_HandleTypeDef *huart);
void K33_Modbus_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void K33_Modbus_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* K33_MODBUS_H */
