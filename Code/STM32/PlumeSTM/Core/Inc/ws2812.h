#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "stm32wbxx_hal.h" // Or your specific HAL header

#define NUM_LEDS 1 // Number of LEDs in your strip/chain
#define WS2812_BYTES_PER_LED (24) // 8 bits for G, 8 for R, 8 for B -> 24 SPI bytes
#define WS2812_RESET_BYTES 60     // For >50us reset pulse (60 bytes * 8 bits/byte * 0.125us/bit = 60us)

#define WS2812_LOGIC_0_CODE 0xE0 // SPI byte for WS2812 logic '0' (11100000)
#define WS2812_LOGIC_1_CODE 0xFC // SPI byte for WS2812 logic '1' (11111100)

// Buffer to hold all SPI data for LEDs + reset pulse
extern uint8_t ws2812_led_buffer[WS2812_BYTES_PER_LED * NUM_LEDS + WS2812_RESET_BYTES];
extern volatile uint8_t ws2812_spi_tx_complete;

void ws2812_init(SPI_HandleTypeDef *hspi_handle);
void ws2812_set_led_color(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_all_leds_color(uint8_t r, uint8_t g, uint8_t b);
void ws2812_clear_all_leds(void);
void ws2812_show(void);

#endif /* INC_WS2812_H_ */