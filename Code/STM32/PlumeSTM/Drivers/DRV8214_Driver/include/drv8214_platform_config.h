// DRV8214_Driver/Inc/drv8214_platform_config.h
#ifndef DRV8214_PLATFORM_CONFIG_H
#define DRV8214_PLATFORM_CONFIG_H

// Option 1: Automatic Platform Detection (Keep as is if it works for STM32 build tools)
#if defined(ARDUINO) // This is usually defined by the Arduino build system
    #define DRV8214_PLATFORM_ARDUINO
    #include <Arduino.h>
    #include "I2C.h"
#elif defined(STM32WB5Mxx) || defined(STM32F4xx) || defined(USE_STM32_HAL_DRIVER) || defined(USE_HAL_DRIVER)
    #define DRV8214_PLATFORM_STM32
    #include <stdio.h>         // For snprintf
    #include <math.h>
#else
    // Fallback or manual definition
    // #define DRV8214_PLATFORM_STM32 // Uncomment if auto-detection fails for STM32
    // #warning "Platform not auto-detected. Defaulting or manual define needed."
    #error "Unsupported platform. Define DRV8214_PLATFORM_ARDUINO or DRV8214_PLATFORM_STM32 manually or fix auto-detection."
#endif

#endif // DRV8214_PLATFORM_CONFIG_H