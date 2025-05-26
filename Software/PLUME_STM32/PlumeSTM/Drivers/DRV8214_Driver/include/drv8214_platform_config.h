/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the drv8214_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#ifndef DRV8214_PLATFORM_CONFIG_H
#define DRV8214_PLATFORM_CONFIG_H

// Automatic Platform Detection
#if defined(ESP32) || defined(ESP_PLATFORM) || defined(ARDUINO)
    #define DRV8214_PLATFORM_ARDUINO
    #include <Arduino.h>
    #include "I2C.h"
#elif defined(STM32WB5Mxx) || defined(STM32F4xx) || defined(USE_STM32_HAL_DRIVER) || defined(USE_HAL_DRIVER)
    #define DRV8214_PLATFORM_STM32
    #include <stdio.h>         // For snprintf
    #include <math.h>
#else
    #error "Unsupported platform. Define DRV8214_PLATFORM_ARDUINO or DRV8214_PLATFORM_STM32 manually or fix auto-detection."
#endif

#endif // DRV8214_PLATFORM_CONFIG_H