/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the drv8214_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#include "drv8214_platform_i2c.h"

#ifdef DRV8214_PLATFORM_STM32
    static I2C_HandleTypeDef* drv_i2c_handle = NULL; // Static pointer to the I2C handle

    void drv8214_i2c_set_handle(I2C_HandleTypeDef* hi2c) {
        drv_i2c_handle = hi2c;
    }
#endif

void drv8214_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value) {
    if (drv_i2c_handle == NULL) {
        // Handle error: I2C handle not set
        return;
    }
#ifdef DRV8214_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#elif defined(DRV8214_PLATFORM_STM32)
    uint8_t data[2] = { reg, value };
    // STM32 HAL expects the 7-bit address to be shifted left by 1
    HAL_I2C_Master_Transmit(drv_i2c_handle, (uint16_t)(device_address << 1), data, 2, HAL_MAX_DELAY);
    // Add error handling for HAL_StatusTypeDef if needed
#endif
}

uint8_t drv8214_i2c_read_register(uint8_t device_address, uint8_t reg) {
    if (drv_i2c_handle == NULL) {
         // Handle error: I2C handle not set
        return 0;
    }
#ifdef DRV8214_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    Wire.endTransmission(false); // Send restart condition
    Wire.requestFrom(device_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0; // Error or no data
#elif defined(DRV8214_PLATFORM_STM32)
    uint8_t data = 0;
    // STM32 HAL I2C typically uses separate Transmit then Receive for this,
    // or HAL_I2C_Mem_Read for register-based reads.
    // Your Arduino code pattern translates better to separate Transmit/Receive.
    if (HAL_I2C_Master_Transmit(drv_i2c_handle, (uint16_t)(device_address << 1), &reg, 1, HAL_MAX_DELAY) == HAL_OK) {
        if (HAL_I2C_Master_Receive(drv_i2c_handle, (uint16_t)(device_address << 1), &data, 1, HAL_MAX_DELAY) == HAL_OK) {
            return data;
        }
    }
    // Consider using HAL_I2C_Mem_Read for more robustness:
    // HAL_I2C_Mem_Read(drv_i2c_handle, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return 0; // Error
#endif
}

void drv8214_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits) {
    uint8_t current_value = drv8214_i2c_read_register(device_address, reg);
    if (enable_bits) {
        current_value |= mask;  // Set bits
    } else {
        current_value &= ~mask; // Clear bits
    }
    drv8214_i2c_write_register(device_address, reg, current_value);
}

void drv8214_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value) {
    uint8_t current_value = drv8214_i2c_read_register(device_address, reg);
    current_value = (current_value & ~mask) | (new_value & mask); // Apply new value only to masked bits
    drv8214_i2c_write_register(device_address, reg, current_value);
}