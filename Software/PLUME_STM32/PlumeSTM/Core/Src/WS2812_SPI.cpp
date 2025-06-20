/*
 * WS2812_SPI.cpp
 *
 *  Created on: Sep 4, 2023
 *  Author: controllerstech
 */

#include "main.h"
#include "WS2812_SPI.h"

// SPI Handle - make sure this matches what CubeMX generated for SPI2
extern SPI_HandleTypeDef hspi2;

// Static variable to track if 5V power for the LED is currently on
static uint8_t s_is_led_power_on = 0;

// Internal helper function to send the SPI data for one LED
static void ws2812_send_spi_data(uint8_t r_scaled, uint8_t g_scaled, uint8_t b_scaled) {
    uint32_t color_data = ((uint32_t)g_scaled << 16) | ((uint32_t)r_scaled << 8) | (uint32_t)b_scaled;
    uint8_t spi_tx_buffer[24]; // Buffer to hold the 24 * 3-bit encoded values

    // This encoding is for the Controllerstech method with ~2.4MHz SPI clock
    for (int i = 0; i < 24; i++) { // Iterate 24 times for 24 WS2812B data bits
        if ((color_data >> (23 - i)) & 0x01) { // Check MSB first
            spi_tx_buffer[i] = 0b00000110; // WS2812B '1' (last 3 bits are 110)
        } else {
            spi_tx_buffer[i] = 0b00000100; // WS2812B '0' (last 3 bits are 100)
        }
    }

    // Blocking SPI transmit
    HAL_SPI_Transmit(&hspi2, spi_tx_buffer, 24, 100); // 100ms timeout
}

/**
 * @brief Sets the color and brightness of the WS2812B LED.
 *
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param brightness_val Brightness (0-100). If 0, LED will be turned off and power disabled.
 */
void WS2812_SetColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness_val) {
    uint8_t r_scaled, g_scaled, b_scaled;

    if (brightness_val == 0 || (r == 0 && g == 0 && b == 0)) {
        // Turn LED completely OFF and disable 5V power
        if (s_is_led_power_on) {
            // Send all zeros to ensure LED is off before cutting power,
            // though just cutting power after last command would also work.
            // This is more explicit.
            ws2812_send_spi_data(0, 0, 0);
            HAL_Delay(1); // Ensure data is latched/reset (MOSI idle low > 50us)
            HAL_GPIO_WritePin(LED_5V_EN_GPIO_Port, LED_5V_EN_Pin, GPIO_PIN_RESET);
            s_is_led_power_on = 0;
        }
        return; // Nothing more to do if brightness is 0 or color is black
    }

    // Ensure 5V power is ON if it wasn't already
    if (!s_is_led_power_on) {
        HAL_GPIO_WritePin(LED_5V_EN_GPIO_Port, LED_5V_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(1); // Allow power to stabilize (adjust if too long/short)
        s_is_led_power_on = 1;
    }

    // Clamp brightness
    if (brightness_val > 100) {
        brightness_val = 100;
    }

    // Scale colors by brightness
    // (uint32_t) cast is important to avoid overflow during multiplication before division
    r_scaled = (uint8_t)(((uint32_t)r * brightness_val) / 100);
    g_scaled = (uint8_t)(((uint32_t)g * brightness_val) / 100);
    b_scaled = (uint8_t)(((uint32_t)b * brightness_val) / 100);

    ws2812_send_spi_data(r_scaled, g_scaled, b_scaled);

    // After sending data, a low signal on DIN for >50µs is needed to latch the data
    // and reset the internal shift registers for the next data.
    // HAL_SPI_Transmit is blocking. After it finishes, the SPI MOSI line will
    // typically return to its idle state (which should be low for CPOL=Low).
    // A HAL_Delay(1) is much longer than 50µs and will ensure this latch/reset.
    HAL_Delay(1);
    // Power (PC2) remains ON until explicitly turned off by calling with brightness 0 or black.
}
