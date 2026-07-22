/**
 ******************************************************************************
 * @file    k33_task.c
 ******************************************************************************
 */
#include <string.h>
#include "k33_task.h"

static K33_SharedData_t s_shared;
static osMutexId_t      s_dataMutex;
static osThreadId_t     s_taskHandle;

static void k33_task_fn(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint16_t status_bits = 0;
        K33_CO2Reading_t co2 = {0};

        K33_Status_t st = K33_ReadStatusAndCO2(&status_bits, &co2);

        if (osMutexAcquire(s_dataMutex, pdMS_TO_TICKS(50)) == osOK)
        {
            s_shared.last_error = st;
            s_shared.last_update_tick = HAL_GetTick();
            if (st == K33_OK)
            {
                s_shared.status_bits = status_bits;
                s_shared.co2 = co2;
                s_shared.data_valid = true;
            }
            /* on error we deliberately keep the previous good reading in
             * s_shared.co2 / status_bits - only last_error / last_update_tick
             * change, so the GUI can show "stale" rather than garbage. */
            osMutexRelease(s_dataMutex);
        }

        osDelay(pdMS_TO_TICKS(K33_TASK_PERIOD_MS));
    }
}

K33_Status_t K33_Task_Start(void)
{
    const osMutexAttr_t mutexAttr = { .name = "k33DataMtx" };
    s_dataMutex = osMutexNew(&mutexAttr);
    if (s_dataMutex == NULL)
    {
        return K33_ERR_UART;
    }

    memset(&s_shared, 0, sizeof(s_shared));

    const osThreadAttr_t taskAttr = {
        .name = "k33Task",
        .stack_size = K33_TASK_STACK_SIZE_WORDS * sizeof(uint32_t),
        .priority = (osPriority_t)K33_TASK_PRIORITY,
    };
    s_taskHandle = osThreadNew(k33_task_fn, NULL, &taskAttr);
    if (s_taskHandle == NULL)
    {
        return K33_ERR_UART;
    }

    return K33_OK;
}

bool K33_Task_GetLatest(K33_SharedData_t *out)
{
    if (out == NULL) { return false; }

    bool valid = false;
    if (osMutexAcquire(s_dataMutex, pdMS_TO_TICKS(20)) == osOK)
    {
        *out = s_shared;
        valid = s_shared.data_valid;
        osMutexRelease(s_dataMutex);
    }
    return valid;
}
