/**
 ******************************************************************************
 * @file    k33_sensor.c
 ******************************************************************************
 */
#include <string.h>
#include "k33_sensor.h"

/* Register addresses = (register number - 1), per datasheet convention */
#define REG_IR_METER_STATUS   0x0000u  /* IR1 */
#define REG_IR_SPACE_CO2      0x0003u  /* IR4 */
#define REG_HR_ACK            0x0000u  /* HR1 */
#define REG_HR_SPECIAL_CMD    0x0001u  /* HR2 */
#define REG_HR_ABC_PERIOD     0x001Fu  /* HR32 */

#define SPECIAL_CMD_CALIBRATE       0x7Cu
#define SPECIAL_CMD_PARAM_BACKGROUND 0x06u
#define SPECIAL_CMD_PARAM_ZERO        0x07u

static K33_Status_t read_registers(uint8_t func_code, uint16_t start_addr, uint16_t qty,
                                    uint8_t *out_data, uint16_t out_data_cap, uint16_t *out_bytes)
{
    uint8_t req[6] = {
        K33_SLAVE_ADDRESS, func_code,
        (uint8_t)(start_addr >> 8), (uint8_t)(start_addr & 0xFF),
        (uint8_t)(qty >> 8),        (uint8_t)(qty & 0xFF),
    };
    uint8_t resp[K33_MAX_FRAME_LEN];
    uint16_t expLen = 3u + (2u * qty) + 2u; /* addr+func+bytecount + data + crc */
    uint16_t actLen = 0;

    K33_Status_t st = K33_Modbus_Transact(req, sizeof(req), resp, sizeof(resp), expLen, &actLen);
    if (st != K33_OK)
    {
        return st;
    }

    uint8_t byteCount = resp[2];
    if ((byteCount > out_data_cap) || ((3u + byteCount + 2u) > actLen))
    {
        return K33_ERR_LEN;
    }
    memcpy(out_data, &resp[3], byteCount);
    if (out_bytes) { *out_bytes = byteCount; }
    return K33_OK;
}

static K33_Status_t write_register(uint16_t addr, uint16_t value)
{
    uint8_t req[6] = {
        K33_SLAVE_ADDRESS, 0x06u,
        (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF),
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF),
    };
    uint8_t resp[K33_MAX_FRAME_LEN];
    uint16_t actLen = 0;
    /* function 06 response is an echo: addr+func+addrHi+addrLo+valHi+valLo+crc(2) = 8 bytes */
    return K33_Modbus_Transact(req, sizeof(req), resp, sizeof(resp), 8u, &actLen);
}

static float scale_co2(uint16_t raw)
{
    return (float)raw / (float)K33_CO2_RAW_DIVISOR;
}

K33_Status_t K33_Sensor_Init(void)
{
    return K33_OK;
}

K33_Status_t K33_ReadCO2(K33_CO2Reading_t *out)
{
    if (out == NULL) { return K33_ERR_PARAM; }
    uint8_t data[2];
    uint16_t nbytes = 0;
    K33_Status_t st = read_registers(0x04u, REG_IR_SPACE_CO2, 1u, data, sizeof(data), &nbytes);
    if (st != K33_OK) { return st; }
    if (nbytes != 2u) { return K33_ERR_LEN; }

    out->raw = ((uint16_t)data[0] << 8) | data[1];
    out->value = scale_co2(out->raw);
    return K33_OK;
}

K33_Status_t K33_ReadStatus(uint16_t *status_bits)
{
    if (status_bits == NULL) { return K33_ERR_PARAM; }
    uint8_t data[2];
    uint16_t nbytes = 0;
    K33_Status_t st = read_registers(0x04u, REG_IR_METER_STATUS, 1u, data, sizeof(data), &nbytes);
    if (st != K33_OK) { return st; }
    if (nbytes != 2u) { return K33_ERR_LEN; }

    *status_bits = ((uint16_t)data[0] << 8) | data[1];
    return K33_OK;
}

K33_Status_t K33_ReadStatusAndCO2(uint16_t *status_bits, K33_CO2Reading_t *co2)
{
    if ((status_bits == NULL) || (co2 == NULL)) { return K33_ERR_PARAM; }
    /* IR1..IR4 in one transaction, per datasheet Appendix A "Sensor status and CO2 read sequence" */
    uint8_t data[8];
    uint16_t nbytes = 0;
    K33_Status_t st = read_registers(0x04u, REG_IR_METER_STATUS, 4u, data, sizeof(data), &nbytes);
    if (st != K33_OK) { return st; }
    if (nbytes != 8u) { return K33_ERR_LEN; }

    *status_bits = ((uint16_t)data[0] << 8) | data[1];               /* IR1 */
    co2->raw     = ((uint16_t)data[6] << 8) | data[7];                /* IR4 */
    co2->value   = scale_co2(co2->raw);
    return K33_OK;
}

K33_Status_t K33_ClearAckRegister(void)
{
    return write_register(REG_HR_ACK, 0x0000u);
}

K33_Status_t K33_StartBackgroundCalibration(void)
{
    uint16_t cmd = ((uint16_t)SPECIAL_CMD_CALIBRATE << 8) | SPECIAL_CMD_PARAM_BACKGROUND;
    return write_register(REG_HR_SPECIAL_CMD, cmd);
}

K33_Status_t K33_StartZeroCalibration(void)
{
    uint16_t cmd = ((uint16_t)SPECIAL_CMD_CALIBRATE << 8) | SPECIAL_CMD_PARAM_ZERO;
    return write_register(REG_HR_SPECIAL_CMD, cmd);
}

K33_Status_t K33_CheckCalibrationAck(K33_AckBit_t bit, bool *is_set)
{
    if (is_set == NULL) { return K33_ERR_PARAM; }
    uint8_t data[2];
    uint16_t nbytes = 0;
    /* HR1 read via function 03 (Read Holding Registers) */
    K33_Status_t st = read_registers(0x03u, REG_HR_ACK, 1u, data, sizeof(data), &nbytes);
    if (st != K33_OK) { return st; }
    if (nbytes != 2u) { return K33_ERR_LEN; }

    uint16_t ack = ((uint16_t)data[0] << 8) | data[1];
    *is_set = (ack & (uint16_t)bit) != 0;
    return K33_OK;
}

K33_Status_t K33_GetABCPeriod(uint16_t *hours)
{
    if (hours == NULL) { return K33_ERR_PARAM; }
    uint8_t data[2];
    uint16_t nbytes = 0;
    K33_Status_t st = read_registers(0x03u, REG_HR_ABC_PERIOD, 1u, data, sizeof(data), &nbytes);
    if (st != K33_OK) { return st; }
    if (nbytes != 2u) { return K33_ERR_LEN; }

    *hours = ((uint16_t)data[0] << 8) | data[1];
    return K33_OK;
}

K33_Status_t K33_SetABCPeriod(uint16_t hours)
{
    return write_register(REG_HR_ABC_PERIOD, hours);
}
