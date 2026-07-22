/**
 ******************************************************************************
 * @file    k33_config.h
 * @brief   Project-side configuration for the Senseair K33 Modbus library.
 *          Edit THIS file per project. Nothing in k33_modbus.* / k33_sensor.*
 *          / k33_task.* hardcodes a peripheral or a comm strategy.
 ******************************************************************************
 */
#ifndef K33_CONFIG_H
#define K33_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"          /* brings in UART_HandleTypeDef from your CubeMX project */
#include "cmsis_os2.h"      /* FreeRTOS CMSIS-RTOS2 wrapper - swap for FreeRTOS.h/task.h
                                 directly if you use the native API instead */

/* =====================================================================
 * 1) Communication strategy - pick exactly ONE
 * =====================================================================
 * K33_COMM_POLLING  : HAL_UART_Receive() blocking call, executed inside
 *                      the K33 RTOS task (NOT inside an ISR). Simplest,
 *                      costs one task's worth of blocked time per poll.
 *
 * K33_COMM_IT       : HAL_UART_Receive_IT() for a known, exact response
 *                      length (Modbus response length is deterministic
 *                      from the request, see K33_Modbus_Transact()).
 *                      A semaphore is given from HAL_UART_RxCpltCallback.
 *
 * K33_COMM_DMA_IDLE : HAL_UARTEx_ReceiveToIdle_DMA(). Most robust: frame
 *                      end is detected by the UART IDLE line, so it does
 *                      not depend on us predicting the exact byte count.
 *                      Recommended default for FreeRTOS + H7.
 * ===================================================================== */
#define K33_COMM_POLLING     1
#define K33_COMM_IT          2
#define K33_COMM_DMA_IDLE    3

#define K33_COMM_MODE        K33_COMM_DMA_IDLE   /* <-- choose one of the three above */

/* =====================================================================
 * 2) Modbus / bus parameters
 * ===================================================================== */
#define K33_SLAVE_ADDRESS          0xFEu   /* 0xFE = "any sensor" - single sensor on the bus.
                                               Use 1..247 if you address it individually. */
#define K33_RESPONSE_TIMEOUT_MS    180u    /* datasheet TDE2336 sec. 5, "Response time-out" */
#define K33_MAX_FRAME_LEN          28u     /* datasheet sec. 1.1, max PDU incl. addr + CRC */

/* =====================================================================
 * 3) Sensor scaling
 * =====================================================================
 * IMPORTANT: the datasheet only documents the /10 scale factor for the
 * K33 ICB (low, e.g. 0-10000ppm) variant. A 30% full-scale K33 reports
 * a range (0-300000ppm) that cannot fit raw in a 16-bit register, so the
 * firmware almost certainly reports it in a different unit (candidates:
 * 0.01% resolution -> raw/100 = %, or ppm/10, or ppm/100). This is NOT
 * stated in TDE2336 for the 30% variant - verify on the bench with a
 * known reference gas (or contact Senseair) before trusting readings,
 * then set the divisor below. K33_ReadCO2() always returns raw AND the
 * scaled value so you can sanity-check this on first bring-up.
 * ===================================================================== */
#define K33_CO2_RAW_DIVISOR        100.0f   /* raw_register / DIVISOR = physical unit (verify!) */
#define K33_CO2_UNIT_IS_PERCENT    1        /* 1 = result is %CO2, 0 = result is ppm */

/* =====================================================================
 * 4) RTOS task (see k33_task.c)
 * ===================================================================== */
#define K33_TASK_PERIOD_MS         2000u    /* datasheet recommends status polling ~2s */
#define K33_TASK_STACK_SIZE_WORDS  384u
#define K33_TASK_PRIORITY          osPriorityNormal

#ifdef __cplusplus
}
#endif

#endif /* K33_CONFIG_H */
