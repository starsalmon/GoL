#pragma once

// Board selection is done via PlatformIO build flags.
// Default (no BOARD_FNK0104): Unexpected Maker ProS3 + 128x128 SSD1351 OLED.

#if defined(BOARD_FNK0104)
// Freenove FNK0104B (2.8" ILI9341 + FT6336 capacitive touch)
// Label example: "FNK0104 - B - A1B0"
// Official refs: Freenove docs + xiaozhi board config for the 2.8" ILI9341.

#define SCREEN_W        320
#define SCREEN_H        240
#define GRID_W          160
#define GRID_H          120
#define CELL_SIZE       2

#define NEOPIXEL_PIN    42          // official FNK0104A/B WS2812 pin (N/S use 40)
#define BAT_ADC_PIN     9           // official 2:1 divider on the 2.8" board
#define BAT_DIVIDER     2.0f
#define BOOT_BTN_PIN    0

#define HAS_TOUCH       1
#define TOUCH_SDA       16
#define TOUCH_SCL       15
#define TOUCH_INT       17
#define TOUCH_RST       18
#define TOUCH_I2C_ADDR  0x38

#else
// Unexpected Maker ProS3 + SSD1351 128x128 OLED

#define LDO2_EN         17          // enables the OLED supply
#define OLED_CS         5
#define OLED_DC         4
#define OLED_RST        2

#define SCREEN_W        128
#define SCREEN_H        128
#define GRID_W          128
#define GRID_H          128
#define CELL_SIZE       1

#define NEOPIXEL_PIN    18
#define BAT_ADC_PIN     10
#define BAT_SCALE       3.95f       // calibrated ProS3 divider scale
#define CHARGE_SENSE_PIN 33

#endif
