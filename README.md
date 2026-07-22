# K33 Modbus Library — STM32H743 + FreeRTOS + TouchGFX

Wraps the Modbus-RTU interface described in Senseair **TDE2336 rev.11**
("Modbus on Senseair K30, K33 and eSENSE") for a K33 sensor, and exposes the
CO2 reading to a TouchGFX `Model` through a mutex-protected struct.

## Layout

```
Inc/
  k33_config.h   <- edit this per project (comm mode, scale factor, task period)
  k33_modbus.h   <- generic Modbus RTU transport (transport-agnostic UART)
  k33_sensor.h   <- K33 register map / commands
  k33_task.h     <- FreeRTOS polling task + thread-safe getter
Src/
  k33_modbus.c
  k33_sensor.c
  k33_task.c
```

Nothing in `k33_modbus.c` / `k33_sensor.c` / `k33_task.c` hardcodes a
peripheral — the UART handle is bound once at runtime via
`K33_Modbus_Init(&huartX)`, and you pick whichever `UART_HandleTypeDef`
CubeMX generated for the pins you wired the sensor to.

## 1. CubeMX setup (per chosen `K33_COMM_MODE`)

The sensor talks **9600 8N1**, RTU, no parity (see TDE2336 sec. 2). Set that
on whichever USART/UART you use, regardless of mode.

| Mode | CubeMX NVIC | CubeMX DMA |
|---|---|---|
| `K33_COMM_POLLING` | none needed | none needed |
| `K33_COMM_IT` | enable the USART global interrupt | none needed |
| `K33_COMM_DMA_IDLE` | enable USART global interrupt + DMA stream interrupt | add an RX DMA request (Normal mode) on that USART |

## 2. main.c / freertos.c wiring

```c
/* after MX_USARTx_Init() (and MX_DMA_Init() if using DMA_IDLE mode) */
if (K33_Modbus_Init(&huart3) != K33_OK) { Error_Handler(); }
if (K33_Sensor_Init()        != K33_OK) { Error_Handler(); }
if (K33_Task_Start()         != K33_OK) { Error_Handler(); }
```

## 3. Forward the HAL callbacks

Only needed for `K33_COMM_IT` and `K33_COMM_DMA_IDLE` — harmless to add
either way. Put this in `main.c` (or wherever your project doesn't already
define these) or merge into your existing callbacks if you have other
peripherals using them too:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    K33_Modbus_RxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    K33_Modbus_RxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    K33_Modbus_ErrorCallback(huart);
}
```

## 4. TouchGFX Model

```cpp
// Model.hpp
extern "C" {
  #include "k33_task.h"
}

class Model
{
public:
    void tick();
    float getCO2Value() const { return lastCO2; }
    bool  isDataStale() const { return !dataValid; }
private:
    float lastCO2 = 0.0f;
    bool  dataValid = false;
};

// Model.cpp
void Model::tick()
{
    K33_SharedData_t data;
    dataValid = K33_Task_GetLatest(&data);
    if (dataValid)
    {
        lastCO2 = data.co2.value; // % or ppm depending on K33_CO2_UNIT_IS_PERCENT
    }
    // Presenter can also surface data.status_bits / data.last_error for a
    // fault icon (bit-test against K33_StatusBit_t).
}
```

`Model::tick()` runs on the TouchGFX/GUI task and only takes a short-lived
mutex + struct copy — it never touches the UART, so it can't block on the
sensor.

## 5. Calibration example

```c
K33_ClearAckRegister();
K33_StartBackgroundCalibration();
osDelay(pdMS_TO_TICKS(2000)); /* datasheet: wait >= one lamp cycle, ~2s */
bool done = false;
K33_CheckCalibrationAck(K33_ACK_BACKGROUND_CAL_DONE, &done);
```

## 6. ⚠️ CO2 scale factor — verify before trusting readings

TDE2336 only documents a `/10` scale for the **K33 ICB** (low range)
variant's IR4 register. It does not document the scaling for a 30%
full-scale range (300000 ppm), which cannot fit raw into a 16-bit register,
so the sensor firmware necessarily reports it in a different unit. `k33_sensor.c`
exposes both the raw register (`K33_CO2Reading_t.raw`) and the value scaled
by `K33_CO2_RAW_DIVISOR` from `k33_config.h` (default guess: raw/100 = %CO2)
specifically so you can compare `raw` against a reference gas on the bench
and correct the divisor/unit flag before relying on it in the GUI.

## 7. Notes from the datasheet worth keeping in mind

- Max Modbus PDU the sensor accepts is 28 bytes (`K33_MAX_FRAME_LEN`).
- Max registers per read command is 8 (already respected: this library only
  ever requests small counts).
- Response timeout budget is 180 ms (`K33_RESPONSE_TIMEOUT_MS`).
- Address `0xFE` = "any sensor", for a single-slave bus only — don't use it
  if you ever put a second sensor on the same bus.
