/**
 ******************************************************************************
 * @file    k33_sensor.h
 * @brief   K33-specific register access, built on k33_modbus.
 *          Register map per Senseair TDE2336 rev 11 ("Modbus on Senseair
 *          K30, K33 and eSENSE"), sections 3 & 7.
 ******************************************************************************
 */
#ifndef K33_SENSOR_H
#define K33_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "k33_modbus.h"

/* MeterStatus (IR1) bit meanings, datasheet Table 3 */
typedef enum
{
    K33_STATUS_FATAL_ERROR       = (1u << 0),
    K33_STATUS_OFFSET_REG_ERROR  = (1u << 1),
    K33_STATUS_ALGORITHM_ERROR   = (1u << 2),
    K33_STATUS_OUTPUT_ERROR      = (1u << 3),
    K33_STATUS_SELF_DIAG_ERROR   = (1u << 4),
    K33_STATUS_OUT_OF_RANGE      = (1u << 5),
    K33_STATUS_MEMORY_ERROR      = (1u << 6),
} K33_StatusBit_t;

/* Acknowledgement register (HR1) bit meanings, datasheet Table 4 */
typedef enum
{
    K33_ACK_BACKGROUND_CAL_DONE = (1u << 5), /* CI6 */
    K33_ACK_NITROGEN_ZERO_CAL_DONE = (1u << 6), /* CI7 - datasheet table labels this
                                                     "nitrogen calibration" but the prose
                                                     calibration-sequence example refers to
                                                     command param 0x7 as "zero calibration" -
                                                     the two are the same operation, datasheet
                                                     is inconsistent on the name only. */
} K33_AckBit_t;

typedef struct
{
    uint16_t raw;     /* raw register value as returned by the sensor */
    float    value;   /* scaled per K33_CO2_RAW_DIVISOR / K33_CO2_UNIT_IS_PERCENT - VERIFY on bench */
} K33_CO2Reading_t;

/**
 * @brief  Must be called after K33_Modbus_Init(). No sensor-specific state
 *         beyond the transport today, kept as a hook for future extension.
 */
K33_Status_t K33_Sensor_Init(void);

/** Read Space CO2 (IR4, function 04). */
K33_Status_t K33_ReadCO2(K33_CO2Reading_t *out);

/** Read MeterStatus (IR1, function 04). Compare bits against K33_StatusBit_t. */
K33_Status_t K33_ReadStatus(uint16_t *status_bits);

/** Read CO2 (IR4) + status (IR1) in a single Modbus transaction (IR1..IR4). */
K33_Status_t K33_ReadStatusAndCO2(uint16_t *status_bits, K33_CO2Reading_t *co2);

/** Clear the acknowledgement register (HR1) before requesting a calibration. */
K33_Status_t K33_ClearAckRegister(void);

/** Write the Special Command Register (HR2) to start a background calibration. */
K33_Status_t K33_StartBackgroundCalibration(void);

/** Write the Special Command Register (HR2) to start a zero/nitrogen calibration. */
K33_Status_t K33_StartZeroCalibration(void);

/** Read HR1 and report whether the given K33_AckBit_t is set. */
K33_Status_t K33_CheckCalibrationAck(K33_AckBit_t bit, bool *is_set);

/** ABC (automatic background calibration) period, HR32, in hours. 0 = disabled. */
K33_Status_t K33_GetABCPeriod(uint16_t *hours);
K33_Status_t K33_SetABCPeriod(uint16_t hours);
static inline K33_Status_t K33_DisableABC(void) { return K33_SetABCPeriod(0); }

#ifdef __cplusplus
}
#endif

#endif /* K33_SENSOR_H */
