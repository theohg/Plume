#ifndef BMI2_USER_INTERFACE_H_
#define BMI2_USER_INTERFACE_H_

#include "stm32wbxx_hal.h"
#include "bmi2.h"          // For BMI2_INTF_RETURN_TYPE

// Define your BMI270 I2C Address (7-bit)
// Check your schematic/datasheet. Common addresses are 0x68 or 0x69.
#define BMI270_I2C_ADDR_PRIM (0x68) // Example if SDO is LOW
#define BMI270_I2C_ADDR_SEC  (0x69) // Example if SDO is HIGH
#define BMI270_I2C_ADDR      BMI270_I2C_ADDR_PRIM // Choose the one you use

#ifdef __cplusplus  // This is the crucial part for C++ compatibility
extern "C" {
#endif

// Function to set the I2C handle for BMI270 communication
void bmi2_set_i2c_handle(I2C_HandleTypeDef* hi2c_bmi);

// Platform-specific I2C read function
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

// Platform-specific I2C write function
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

// Platform-specific delay function (microseconds)
void bmi2_delay_us(uint32_t period_us, void *intf_ptr);

#ifdef __cplusplus
}
#endif

#endif /* BMI2_USER_INTERFACE_H_ */
