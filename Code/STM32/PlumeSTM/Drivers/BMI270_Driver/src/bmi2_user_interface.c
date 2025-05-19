#include "bmi2_user_interface.h"
#include "i2c.h" // Assuming your I2C_HandleTypeDef hi2c3 is declared here or in main.h

// Static pointer to the I2C handle for BMI270
static I2C_HandleTypeDef* bmi2_i2c_hal_handle = NULL;

void bmi2_set_i2c_handle(I2C_HandleTypeDef* hi2c_bmi) {
    bmi2_i2c_hal_handle = hi2c_bmi;
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    (void)intf_ptr; // intf_ptr is the I2C handle passed via bmi2_dev structure, we use the static one

    if (bmi2_i2c_hal_handle == NULL) {
        return BMI2_E_NULL_PTR; // Or a specific error for uninitialized handle
    }

    // STM32 HAL I2C read function typically requires the device address to be shifted left by 1
    // The HAL_I2C_Mem_Read function is suitable here.
    if (HAL_I2C_Mem_Read(bmi2_i2c_hal_handle, (uint16_t)(BMI270_I2C_ADDR << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY) == HAL_OK) {
        return BMI2_OK;
    } else {
        return BMI2_E_COM_FAIL;
    }
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    (void)intf_ptr; // intf_ptr is the I2C handle passed via bmi2_dev structure, we use the static one

    if (bmi2_i2c_hal_handle == NULL) {
        return BMI2_E_NULL_PTR; // Or a specific error for uninitialized handle
    }
    
    // STM32 HAL I2C write function
    // The HAL_I2C_Mem_Write function is suitable here.
    if (HAL_I2C_Mem_Write(bmi2_i2c_hal_handle, (uint16_t)(BMI270_I2C_ADDR << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, HAL_MAX_DELAY) == HAL_OK) {
        return BMI2_OK;
    } else {
        return BMI2_E_COM_FAIL;
    }
}

void bmi2_delay_us(uint32_t period_us, void *intf_ptr) {
    (void)intf_ptr; // Not used in this simple implementation
    
    // HAL_Delay is in milliseconds. For microsecond delay, a more precise timer is needed.
    // For initial testing, if period_us is large enough:
    if (period_us >= 1000) {
        HAL_Delay(period_us / 1000);
    } else {
        // Crude busy-wait loop for very short delays.
        // This needs calibration for your specific clock speed.
        // Or, use DWT cycle counter or a TIM peripheral for accurate us delay.
        uint32_t i;
        // Assuming a CPU clock around 60MHz, one loop iteration is a few cycles.
        // Calibrate this factor. For 60MHz, roughly 60 cycles per us.
        // Loop overhead will make this approximate.
        uint32_t delay_cycles = period_us * (SystemCoreClock / 1000000U / 5U); // Approximate factor
        for (i = 0; i < delay_cycles; ++i) {
            __NOP();
        }
        if (period_us > 0 && delay_cycles == 0) { // ensure at least minimal delay for very small period_us
             for (i = 0; i < 10; ++i) { // minimal delay
                __NOP();
             }
        }
    }
}