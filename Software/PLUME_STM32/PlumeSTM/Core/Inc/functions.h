#ifndef FUNCTION_H
#define FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h" // Includes CubeMX generated headers like stm32wbxx_hal.h
#include <stdbool.h>
#include <stdio.h>
#include "WS2812_SPI.h"
#include "drv8214.h"
#include "bmi270.h"

/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  WAKEUP_SOURCE_UNKNOWN,
  WAKEUP_SOURCE_IMU,
  WAKEUP_SOURCE_BUTTON,
  WAKEUP_SOURCE_RTC
} WakeupSource_t;

/* Defines -------------------------------------------------------------------*/
// ADC settings
#define VREF_MCU                    3.3f
#define ADC_RESOLUTION              4095.0f
#define R502_VALUE                  3000.0f
#define R503_VALUE                  10000.0f
#define BATTERY_SENSE_DIVIDER_RATIO ((R502_VALUE + R503_VALUE) / R503_VALUE)

// Driver and motor configuration
#define NUM_DRIVERS 2
#define IPROPI_RESISTOR 680
#define NUM_RIPPLES 6
#define MOTOR_INTERNAL_RESISTANCE 20
#define MOTOR_REDUCTION_RATIO 26
#define MAX_MOTOR_RPM 1154

// IMU
#define INACTIVITY_TIMEOUT_MS 1000

// prinf destination
#define USE_SWD

/* Extern Global Variables ---------------------------------------------------*/
// These variables are defined in function.c
extern volatile bool wakeup_event;
extern volatile uint8_t button_ID;
extern volatile bool inactivity_timer_elapsed_flag;
extern volatile bool woke_up_from_stop_mode;
extern volatile uint8_t g_measure_battery_flag;

extern WakeupSource_t g_wakeup_source;
extern DRV8214 drivers[NUM_DRIVERS];
extern uint8_t i2c_channel_to_use;
extern struct bmi2_dev bmi270_sensor;


/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief High-level application initialization function.
 *        Initializes all peripherals, drivers, and application states.
 */
void App_Init(void);

/**
 * @brief High-level application processing function.
 *        This should be called continuously in the main loop.
 */
void App_Process(void);

/**
 * @brief Checks the cause of wakeup from Standby mode.
 */
void CheckWakeupSource(void);


/* Low Power Functions */
void EnterStop2Mode(void);
void EnterStandbyMode(void);
void ResetInactivityTimer(void);
void Wakeup_Reinit_Peripherals(void);


/* Debug/Utility Functions */
void I2C_Scan(I2C_HandleTypeDef *hi2c);
void printRegisters(uint8_t driver_id);


#ifdef __cplusplus
}
#endif

#endif /* FUNCTION_H */