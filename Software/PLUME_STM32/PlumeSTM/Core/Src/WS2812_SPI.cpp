/*
 * WS2812_SPI.cpp
 *
 *  Created on: Sep 4, 2023
 *  Author: controllerstech
 */

#include "main.h"
#include "WS2812_SPI.h"

// --- External HAL Handles ---
extern SPI_HandleTypeDef hspi2;

/* Private Global Variables */ 
LED_State_t g_led_state;
static uint8_t s_is_led_power_on = 0; // to track if 5V power for the LED is currently on

/* Private Function Prototypes */
static void ws2812_send_spi_data(uint8_t r_scaled, uint8_t g_scaled, uint8_t b_scaled);

// --- Public Function Implementations ---

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
            //HAL_Delay(1); // Ensure data is latched/reset (MOSI idle low > 50us)
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
    if (brightness_val > 100) { brightness_val = 100; }

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
    //HAL_Delay(1);
    // Power (PC2) remains ON until explicitly turned off by calling with brightness 0 or black.
}

void Update_LED_State(void) {
    uint32_t current_tick = HAL_GetTick();

    // Check if the current mode is a timed effect and if its time is up.
    if (g_led_state.mode == LED_STATE_TIMED_EFFECT) {
        if (current_tick >= g_led_state.effect_end_time) {
            Set_LED_Off(); // The effect's duration has expired, turn it off.
            return;        // And we're done.
        }
    }

    // If we're here, we process any blinking logic (for both permanent and timed blinks)
    // This logic works for both LED_STATE_BLINKING and LED_STATE_TIMED_EFFECT
    if (g_led_state.mode == LED_STATE_BLINKING || g_led_state.mode == LED_STATE_TIMED_EFFECT) {
        if (current_tick >= g_led_state.next_toggle_time) {
            if (g_led_state.is_on) {
                // Turn it off
                WS2812_SetColor(0, 0, 0, 0);
                g_led_state.is_on = false;
                g_led_state.next_toggle_time = current_tick + g_led_state.off_time_ms;
            } else {
                // Turn it on
                uint8_t r = (g_led_state.color >> 16) & 0xFF;
                uint8_t g = (g_led_state.color >> 8) & 0xFF;
                uint8_t b = g_led_state.color & 0xFF;
                WS2812_SetColor(r, g, b, 100);
                g_led_state.is_on = true;
                g_led_state.next_toggle_time = current_tick + g_led_state.on_time_ms;
            }
        }
    }
    // No action is needed for LED_STATE_OFF or LED_STATE_SOLID
}

// Permanent Blink
void Set_LED_Blink(Color_t color, uint16_t on_time, uint16_t off_time) {
    g_led_state.mode = LED_STATE_BLINKING; // Permanent mode
    g_led_state.color = color;
    g_led_state.on_time_ms = on_time;
    g_led_state.off_time_ms = off_time;
    g_led_state.is_on = false;
    g_led_state.next_toggle_time = HAL_GetTick();
}

// Permanent Solid
void Set_LED_Solid(Color_t color) {
    g_led_state.mode = LED_STATE_SOLID; // Permanent mode
    g_led_state.color = color;
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    WS2812_SetColor(r, g, b, 100);
}

/**
 * @brief Sets the LED to a solid color for a specific duration.
 * @param color The color to display.
 * @param duration_ms The total time in milliseconds to keep the color on.
 */
void Set_LED_Solid_For(Color_t color, uint16_t duration_ms) {
    // A timed solid is just a special case of a timed blink.
    // It blinks "on" for the full duration and never turns "off".
    Set_LED_Blink_For(color, duration_ms, 0, duration_ms);
}

/**
 * @brief Blinks the LED for a specific total duration.
 * @param color The color to blink.
 * @param on_time The "on" time for each blink cycle.
 * @param off_time The "off" time for each blink cycle.
 * @param total_duration_ms The total time the blinking effect should last.
 */
void Set_LED_Blink_For(Color_t color, uint16_t on_time, uint16_t off_time, uint16_t total_duration_ms) {
    g_led_state.mode = LED_STATE_TIMED_EFFECT; // Timed mode
    g_led_state.color = color;
    g_led_state.on_time_ms = on_time;
    g_led_state.off_time_ms = off_time;
    g_led_state.is_on = false; // Start in off state
    uint32_t current_tick = HAL_GetTick();
    g_led_state.next_toggle_time = current_tick; // Toggle immediately
    g_led_state.effect_end_time = current_tick + total_duration_ms;
}

void Set_LED_Off() {
    g_led_state.is_on = false;
    g_led_state.mode = LED_STATE_OFF;
    WS2812_SetColor(0,0,0,0);
}

/**
 * @brief Performs a startup LED sequence to indicate system readiness.
 */
void LED_Startup_Sequence(void) {
    WS2812_SetColor(255, 0, 0, 100); HAL_Delay(250);
    WS2812_SetColor(0, 255, 0, 100); HAL_Delay(250);
    WS2812_SetColor(0, 0, 255, 100); HAL_Delay(250);
    Set_LED_Off();
}