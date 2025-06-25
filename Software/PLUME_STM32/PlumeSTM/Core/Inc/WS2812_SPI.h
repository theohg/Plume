/*
 * WS2812_SPI.h
 *
 *  Created on: Sep 4, 2023
 *      Author: arunrawat
 */

#ifndef INC_WS2812_SPI_H_
#define INC_WS2812_SPI_H_

typedef enum {
    RED = 0xFF0000,    // Red color in RGB format
    GREEN = 0x00FF00,  // Green color in RGB format
    BLUE = 0x0000FF,   // Blue color in RGB format
    WHITE = 0xFFFFFF,  // White color in RGB format
    YELLOW = 0xFFFF00, // Yellow color in RGB format
    ORANGE = 0xFFA500, // Orange color in RGB format
    CYAN = 0x00FFFF,   // Cyan color in RGB format
    MAGENTA = 0xFF00FF // Magenta color in RGB format
} Color_t;

typedef enum {
    LED_STATE_OFF,
    LED_STATE_SOLID,
    LED_STATE_BLINKING,
    LED_STATE_TIMED_EFFECT
} LED_Mode_t;

typedef struct {
    LED_Mode_t mode;
    Color_t    color;
    uint16_t   on_time_ms;
    uint16_t   off_time_ms;
    uint32_t   next_toggle_time; 
    uint32_t   effect_end_time;
    bool       is_on;                // Current state in a blink cycle
} LED_State_t;

extern LED_State_t g_led_state;

// --- Public Function Prototypes ---

void WS2812_SetColor (uint8_t RED, uint8_t GREEN, uint8_t BLUE, uint8_t brightness_val);

// High-level state management functions
void Set_LED_Blink(Color_t color, uint16_t on_time, uint16_t off_time);
void Set_LED_Solid(Color_t color);
void Set_LED_Off();
void LED_Startup_Sequence(void);
void Set_LED_Solid_For(Color_t color, uint16_t duration_ms);
void Set_LED_Blink_For(Color_t color, uint16_t on_time, uint16_t off_time, uint16_t total_duration_ms);

// The main update function that must be called from the main loop
void Update_LED_State(void);

#endif /* INC_WS2812_SPI_H_ */