#include "ws2812.h"
#include "main.h" // For LED_5V_EN_GPIO_Port and LED_5V_EN_Pin
#include <string.h> // For memset

uint8_t ws2812_led_buffer[WS2812_BYTES_PER_LED * NUM_LEDS + WS2812_RESET_BYTES];
volatile uint8_t ws2812_spi_tx_complete = 0;
static SPI_HandleTypeDef *g_hspi_ws2812;

void ws2812_init(SPI_HandleTypeDef *hspi_handle) {
    g_hspi_ws2812 = hspi_handle;
    // Pre-fill the reset pulse part of the buffer with zeros (which will be WS2812_LOGIC_0_CODE)
    // The actual color data part will be filled by set_led_color
    // For SPI, a stream of 0s on MOSI is low. To represent a WS2812 reset (long low),
    // we actually need to send SPI bytes that result in a low signal (e.g. 0x00).
    // However, the WS2812_RESET_BYTES are sent *after* color data. The LEDs reset when DIN is low for >50us.
    // The SPI peripheral, when idle or after TxCplt, will typically hold MOSI low if configured so.
    // A simpler approach is to just ensure DMA sends enough "0" color data if a long low pulse is needed via active sending.
    // But the documented method is a delay after data. So, we ensure the buffer for reset is actually 0x00.
    memset(ws2812_led_buffer + (WS2812_BYTES_PER_LED * NUM_LEDS), 0x00, WS2812_RESET_BYTES);
    ws2812_clear_all_leds(); // Initialize buffer to off
}

// Sets the color of a single LED in the buffer
// IMPORTANT: Color order is GRB for WS2812B
// void ws2812_set_led_color(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b) {
//     if (led_index >= NUM_LEDS) {
//         return; // Index out of bounds
//     }

//     uint32_t buffer_offset = led_index * WS2812_BYTES_PER_LED;

//     // Green
//     for (int i = 0; i < 8; i++) {
//         ws2812_led_buffer[buffer_offset + i] = (g & (1 << (7 - i))) ? WS2812_LOGIC_1_CODE : WS2812_LOGIC_0_CODE;
//     }
//     // Red
//     for (int i = 0; i < 8; i++) {
//         ws2812_led_buffer[buffer_offset + 8 + i] = (r & (1 << (7 - i))) ? WS2812_LOGIC_1_CODE : WS2812_LOGIC_0_CODE;
//     }
//     // Blue
//     for (int i = 0; i < 8; i++) {
//         ws2812_led_buffer[buffer_offset + 16 + i] = (b & (1 << (7 - i))) ? WS2812_LOGIC_1_CODE : WS2812_LOGIC_0_CODE;
//     }
// }
void ws2812_set_led_color(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (led_index >= NUM_LEDS) {
        return;
    }

    uint32_t buffer_offset = led_index * WS2812_BYTES_PER_LED;

    // Green - Keep as is (should be all 0xE0 for g=0)
    for (int i = 0; i < 8; i++) {
        ws2812_led_buffer[buffer_offset + i] = (g & (1 << (7 - i))) ? WS2812_LOGIC_1_CODE : WS2812_LOGIC_0_CODE;
    }

    // Red - TEMPORARY MODIFICATION FOR TESTING
    ws2812_led_buffer[buffer_offset + 8 + 0] = WS2812_LOGIC_0_CODE; // Should be 0xE0
    ws2812_led_buffer[buffer_offset + 8 + 1] = WS2812_LOGIC_0_CODE; // Should be 0xE0
    ws2812_led_buffer[buffer_offset + 8 + 2] = WS2812_LOGIC_1_CODE; // FORCE 0xFC
    ws2812_led_buffer[buffer_offset + 8 + 3] = WS2812_LOGIC_1_CODE; // FORCE 0xFC
    ws2812_led_buffer[buffer_offset + 8 + 4] = WS2812_LOGIC_0_CODE; // Should be 0xE0
    ws2812_led_buffer[buffer_offset + 8 + 5] = WS2812_LOGIC_0_CODE; // Should be 0xE0
    ws2812_led_buffer[buffer_offset + 8 + 6] = WS2812_LOGIC_1_CODE; // FORCE 0xFC
    ws2812_led_buffer[buffer_offset + 8 + 7] = WS2812_LOGIC_0_CODE; // Should be 0xE0

    // Blue - Keep as is (should be all 0xE0 for b=0)
    for (int i = 0; i < 8; i++) {
        ws2812_led_buffer[buffer_offset + 16 + i] = (b & (1 << (7 - i))) ? WS2812_LOGIC_1_CODE : WS2812_LOGIC_0_CODE;
    }

}

void ws2812_set_all_leds_color(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        ws2812_set_led_color(i, r, g, b);
    }
}

void ws2812_clear_all_leds(void) {
    ws2812_set_all_leds_color(0, 0, 0);
}

void ws2812_show(void) {
    ws2812_spi_tx_complete = 0;

    // Enable 5V for LEDs (and level shifter if used)
    HAL_GPIO_WritePin(LED_5V_EN_GPIO_Port, LED_5V_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // Small delay for power to stabilize

    if (HAL_SPI_Transmit_DMA(g_hspi_ws2812, ws2812_led_buffer, sizeof(ws2812_led_buffer)) != HAL_OK) {
        // Error handling
        //HAL_GPIO_WritePin(LED_5V_EN_GPIO_Port, LED_5V_EN_Pin, GPIO_PIN_RESET); // Turn off power on error
    }
    // The rest of the operation (waiting and turning off power) happens in HAL_SPI_TxCpltCallback
}

// This callback is invoked when SPI DMA transfer is complete
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == g_hspi_ws2812) {
        // Ensure SPI peripheral has finished sending all bits from its internal FIFO
        while (HAL_SPI_GetState(g_hspi_ws2812) != HAL_SPI_STATE_READY) {
            // Wait for SPI to be ready (all data shifted out)
        }
        // Data sent, turn off 5V supply for LEDs
        //HAL_GPIO_WritePin(LED_5V_EN_GPIO_Port, LED_5V_EN_Pin, GPIO_PIN_RESET);
        ws2812_spi_tx_complete = 1;
    }
}
