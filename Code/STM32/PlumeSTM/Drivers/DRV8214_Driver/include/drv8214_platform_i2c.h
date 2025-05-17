/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the drv8214_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

// DRV8214_Driver/Inc/drv8214_platform_i2c.h
#ifndef DRV8214_PLATFORM_I2C_H
#define DRV8214_PLATFORM_I2C_H

#include "drv8214_platform_config.h" // For platform detection

#ifdef DRV8214_PLATFORM_ARDUINO
    #include <Arduino.h>
    #include <Wire.h>
#endif

#ifdef DRV8214_PLATFORM_STM32
    #include "stm32wbxx_hal.h" // This should be the main HAL include for your MCU. Can be found in main.h
    #include "i2c.h"           // This is the CubeMX generated i2c.h, which declares hi2c1 and MX_I2C1_Init()

    // Function to set the I2C handle for this module to use
    // Call this once during initialization in main.c
    void drv8214_i2c_set_handle(I2C_HandleTypeDef* hi2c);
#endif

// Common I2C function declarations
void drv8214_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value);
uint8_t drv8214_i2c_read_register(uint8_t device_address, uint8_t reg);
void drv8214_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits); // Changed bool to uint8_t
void drv8214_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value);

#endif // DRV8214_PLATFORM_I2C_H
