/**
 ******************************************************************************
 * @file    k33_modbus.c
 * @brief   Generic Modbus-RTU transport implementation.
 ******************************************************************************
 */
#include <string.h>
#include "k33_modbus.h"

static UART_HandleTypeDef *s_huart = NULL;
static osMutexId_t         s_busMutex;      /* one transaction on the bus at a time */
static osSemaphoreId_t     s_rxDoneSem;      /* given by the RX callback, taken by Transact */
static volatile uint16_t   s_rxLen = 0;       /* bytes captured by the callback */

static uint16_t s_userRespCap = 0;   /* expected length for the K33_COMM_IT callback */

/* ------------------------------------------------------------------------ */
/* CRC16 (Modbus), LSB-first as required by the wire format                  */
/* ------------------------------------------------------------------------ */
uint16_t K33_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

K33_Status_t K33_Modbus_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return K33_ERR_PARAM;
    }

    s_huart = huart;

    const osMutexAttr_t mutexAttr = { .name = "k33BusMtx" };
    s_busMutex = osMutexNew(&mutexAttr);

    const osSemaphoreAttr_t semAttr = { .name = "k33RxSem" };
    s_rxDoneSem = osSemaphoreNew(1, 0, &semAttr); /* starts empty */

    if ((s_busMutex == NULL) || (s_rxDoneSem == NULL))
    {
        return K33_ERR_UART;
    }

#if (K33_COMM_MODE == K33_COMM_DMA_IDLE)
    /* Nothing to arm yet - we (re)arm ReceiveToIdle_DMA per-transaction in
     * K33_Modbus_Transact() so a stale/oversized previous frame can't linger. */
#endif

    return K33_OK;
}

/* ------------------------------------------------------------------------ */
/* Transaction                                                               */
/* ------------------------------------------------------------------------ */
K33_Status_t K33_Modbus_Transact(const uint8_t *req, uint16_t req_len,
                                  uint8_t *resp, uint16_t resp_buf_size,
                                  uint16_t exp_resp_len, uint16_t *actual_len)
{
    if (s_huart == NULL)
    {
        return K33_ERR_NOT_INIT;
    }
    if ((req == NULL) || (resp == NULL) || (actual_len == NULL))
    {
        return K33_ERR_PARAM;
    }
    if ((req_len + 2u) > K33_MAX_FRAME_LEN) /* +2 for CRC */
    {
        return K33_ERR_PARAM;
    }

    if (osMutexAcquire(s_busMutex, pdMS_TO_TICKS(K33_RESPONSE_TIMEOUT_MS)) != osOK)
    {
        return K33_ERR_BUSY;
    }

    K33_Status_t status = K33_OK;

    /* ---- build TX frame: req + CRC (low byte first, per datasheet) ---- */
    uint8_t txBuf[K33_MAX_FRAME_LEN];
    memcpy(txBuf, req, req_len);
    uint16_t crc = K33_CRC16(req, req_len);
    txBuf[req_len]     = (uint8_t)(crc & 0xFF);        /* CRC low */
    txBuf[req_len + 1] = (uint8_t)((crc >> 8) & 0xFF); /* CRC high */
    uint16_t txLen = req_len + 2u;

    /* drain any stale semaphore give from a previous timed-out transaction */
    (void)osSemaphoreAcquire(s_rxDoneSem, 0);
    s_rxLen = 0;
    s_userRespCap = exp_resp_len; /* K33_COMM_IT only: exact byte count requested from HAL */

#if (K33_COMM_MODE == K33_COMM_POLLING)
    /* --- simple blocking round trip, executed inside the caller's task --- */
    if (HAL_UART_Transmit(s_huart, txBuf, txLen, K33_RESPONSE_TIMEOUT_MS) != HAL_OK)
    {
        status = K33_ERR_UART;
        goto done;
    }
    {
        uint16_t toRead = (exp_resp_len != 0) ? exp_resp_len : resp_buf_size;
        HAL_StatusTypeDef hs = HAL_UART_Receive(s_huart, resp, toRead, K33_RESPONSE_TIMEOUT_MS);
        if (hs == HAL_TIMEOUT)
        {
            /* HAL_UART_Receive on H7 leaves whatever it collected in RxXferCount;
             * treat a partial-but-nonzero response as a length error, none as timeout. */
            uint16_t got = toRead - s_huart->RxXferCount;
            if (got == 0) { status = K33_ERR_TIMEOUT; goto done; }
            s_rxLen = got;
        }
        else if (hs != HAL_OK)
        {
            status = K33_ERR_UART;
            goto done;
        }
        else
        {
            s_rxLen = toRead;
        }
    }

#elif (K33_COMM_MODE == K33_COMM_IT)
    /* --- interrupt driven, exact expected length required --- */
    if (exp_resp_len == 0 || exp_resp_len > resp_buf_size)
    {
        status = K33_ERR_PARAM;
        goto done;
    }
    if (HAL_UART_Receive_IT(s_huart, resp, exp_resp_len) != HAL_OK)
    {
        status = K33_ERR_UART;
        goto done;
    }
    if (HAL_UART_Transmit(s_huart, txBuf, txLen, K33_RESPONSE_TIMEOUT_MS) != HAL_OK)
    {
        HAL_UART_AbortReceive_IT(s_huart);
        status = K33_ERR_UART;
        goto done;
    }
    if (osSemaphoreAcquire(s_rxDoneSem, pdMS_TO_TICKS(K33_RESPONSE_TIMEOUT_MS)) != osOK)
    {
        HAL_UART_AbortReceive_IT(s_huart);
        status = K33_ERR_TIMEOUT;
        goto done;
    }
    /* s_rxLen was set to exp_resp_len by the callback */

#elif (K33_COMM_MODE == K33_COMM_DMA_IDLE)
    /* --- DMA + IDLE line detection, robust to not knowing exact length --- */
    if (HAL_UARTEx_ReceiveToIdle_DMA(s_huart, resp, resp_buf_size) != HAL_OK)
    {
        status = K33_ERR_UART;
        goto done;
    }
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT); /* half-transfer IRQ not useful here */

    if (HAL_UART_Transmit(s_huart, txBuf, txLen, K33_RESPONSE_TIMEOUT_MS) != HAL_OK)
    {
        HAL_UART_AbortReceive(s_huart);
        status = K33_ERR_UART;
        goto done;
    }
    if (osSemaphoreAcquire(s_rxDoneSem, pdMS_TO_TICKS(K33_RESPONSE_TIMEOUT_MS)) != osOK)
    {
        HAL_UART_AbortReceive(s_huart);
        status = K33_ERR_TIMEOUT;
        goto done;
    }
    /* s_rxLen set to the Size reported by HAL_UARTEx_RxEventCallback */
#else
#error "K33_COMM_MODE not set to a supported value in k33_config.h"
#endif

    /* ---- common validation: min length, address, CRC ---- */
    if (s_rxLen < 5u) /* addr + func + at least 1 byte + crc(2) */
    {
        status = K33_ERR_LEN;
        goto done;
    }
    if (resp[0] != K33_SLAVE_ADDRESS)
    {
        status = K33_ERR_LEN;
        goto done;
    }
    {
        uint16_t rxCrc = K33_CRC16(resp, s_rxLen - 2u);
        uint16_t gotCrc = (uint16_t)resp[s_rxLen - 2u] | ((uint16_t)resp[s_rxLen - 1u] << 8);
        if (rxCrc != gotCrc)
        {
            status = K33_ERR_CRC;
            goto done;
        }
    }
    if (resp[1] & 0x80u) /* exception response: function code with high bit set */
    {
        status = K33_ERR_EXCEPTION;
        goto done;
    }

    *actual_len = s_rxLen;

done:
    osMutexRelease(s_busMutex);
    return status;
}

/* ------------------------------------------------------------------------ */
/* HAL callbacks - forward these from stm32h7xx_it.c / main.c                */
/* ------------------------------------------------------------------------ */
void K33_Modbus_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if (K33_COMM_MODE == K33_COMM_IT)
    if (huart->Instance != s_huart->Instance) return;
    s_rxLen = s_userRespCap; /* exact length was requested up front */
    osSemaphoreRelease(s_rxDoneSem);
#else
    (void)huart;
#endif
}

void K33_Modbus_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if (K33_COMM_MODE == K33_COMM_DMA_IDLE)
    if (huart->Instance != s_huart->Instance) return;
    s_rxLen = Size;
    osSemaphoreRelease(s_rxDoneSem);
#else
    (void)huart; (void)Size;
#endif
}

void K33_Modbus_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart->Instance != s_huart->Instance) return;
    /* Release any waiter so it can time out/retry cleanly instead of hanging;
     * HAL error flags are cleared by HAL's own error handler. */
    osSemaphoreRelease(s_rxDoneSem);
}
