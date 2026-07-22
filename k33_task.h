/**
 ******************************************************************************
 * @file    k33_task.h
 * @brief   FreeRTOS task that periodically polls the K33 and publishes the
 *          latest reading into a mutex-protected struct. TouchGFX's Model
 *          calls K33_Task_GetLatest() from the GUI task's tick - that call
 *          is quick (just a mutex + memcpy) and never touches the UART
 *          itself, so it's safe to call from the TouchGFX/GUI task context.
 ******************************************************************************
 */
#ifndef K33_TASK_H
#define K33_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "k33_sensor.h"

typedef struct
{
    K33_CO2Reading_t co2;
    uint16_t         status_bits;
    bool             data_valid;      /* false until the first successful read */
    K33_Status_t     last_error;      /* result of the most recent poll, K33_OK if it succeeded */
    uint32_t         last_update_tick;
} K33_SharedData_t;

/**
 * @brief  Creates the shared-data mutex and the polling task. Call once,
 *         after K33_Modbus_Init() and K33_Sensor_Init() have both succeeded
 *         (e.g. from your freertos.c MX_FREERTOS_Init() / defaultTask, or
 *         right after MX_USARTx_Init() in main()).
 */
K33_Status_t K33_Task_Start(void);

/**
 * @brief  Non-blocking-ish (bounded by a short mutex wait) copy of the
 *         latest reading. Safe to call from TouchGFX's Model::tick().
 * @return true if data_valid (i.e. at least one successful poll happened).
 */
bool K33_Task_GetLatest(K33_SharedData_t *out);

#ifdef __cplusplus
}
#endif

#endif /* K33_TASK_H */
