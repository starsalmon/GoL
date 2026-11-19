# GoL (Game of Life) — ESP32 + SSD1351

Conway’s Game of Life, rendered full-screen on a 128×128 SSD1351 SPI OLED, with multiple visual styles and a NeoPixel status/battery LED.

## Features

- Multiple styles (white, rainbow, heatmap, neon trails, plasma, fire)
- 1× / 2× / 4× render modes (2×/4× are “chunkier” downsampled modes)
- Auto-reseed on extinction / repeating cycles
- Optional NeoPixel “breathing” indicator (speeds up when charging)
- Simple battery voltage sampling + filtered percentage-to-colour mapping

## Controls (Serial)

Open the serial monitor at 115200 baud (type any key once to print the help):

- **r**: next render mode
- **e**: next style
- **n**: reseed grid
- **p**: pause/resume
- **h** or **?**: show help
- **w/a/s/d**: pan viewport (2× / 4× modes)

## Hardware

- ESP32-S3 board (configured for `um_pros3` in `platformio.ini`)
- SSD1351 128×128 SPI OLED (Adafruit SSD1351 compatible)
- Optional: WS2812/NeoPixel on a GPIO pin
- Optional: battery sense input (via divider) + a “5V present” sense pin

## Pin mapping (current defaults)

Defined in `src/main.cpp`:

- **OLED**
  - `OLED_CS` = GPIO **5**
  - `OLED_DC` = GPIO **4**
  - `OLED_RST` = GPIO **2**
  - SPI bus uses the board’s default `SPI` pins (SCK/MOSI).
- **NeoPixel**
  - Data = GPIO **18** (currently `Adafruit_NeoPixel pixels(1, 18, ...)`)
  - Power enable = GPIO **17** (drives LDO2 enable HIGH in `setup()`)
- **Battery / power sense**
  - VBAT ADC = GPIO **10** (`analogRead(10)`, with a calibrated scale factor in code)
  - 5V sense (digital) = GPIO **33** (`digitalRead(33) == HIGH` means “charging / 5V present”)

If your wiring differs, change the defines and pin numbers at the top of `src/main.cpp`.

## Build / flash (PlatformIO)

From the project root:

```bash
pio run -e um_pros3
pio run -e um_pros3 -t upload
pio device monitor
```

Notes:
- **Port**: update `upload_port` / `monitor_port` in `platformio.ini` to match your device.
- **Dependencies** are managed by PlatformIO via `lib_deps` in `platformio.ini`.

## 3D-printed enclosure

The enclosure model lives here:

- `Enclosure 3D Model/GoL enclosure 1.5 oled.3mf`

`*.3mf` can be opened directly in tools like PrusaSlicer / Bambu Studio (and many CAD/slicer workflows).

## Freenove FNK0104B (2.8" TFT)

A second PlatformIO environment, `freenove_fnk0104`, targets the Freenove ESP32-S3 Display **FNK0104B** (the “B” variant is the 2.8" **ILI9341** panel with **FT6336** capacitive touch; label example `FNK0104 - B - A1B0`). FNK0104A is the same 2.8" panel without touch. Do not use this env for FNK0104N (3.5" ST77922) or FNK0104S (4.0" ST7789/ST7796).

The Game of Life grid is scaled to fill the 320×240 landscape screen: **160×120 cells at 2×2 pixels**. 2× / 4× render modes zoom a viewport the same way as on the 128×128 OLED.

### Extra controls on this board

- **Tap** the screen to reseed
- **Long-press** to pause / resume
- In **1×** mode, swipe left/right to change style, up/down to change render
- In **2× / 4×** mode, swipe to pan the viewport
- **BOOT**: short press = next style, long press = next render
- Serial controls above still work (115200 baud)

The onboard WS2812 (GPIO 42) is the battery / breathing LED. Backlight is GPIO 45 (active HIGH). There is no dedicated 5V-sense pin on this board, so the LED does not speed up when charging.

```bash
pio run -e freenove_fnk0104
pio run -e freenove_fnk0104 -t upload --upload-port /dev/cu.usbmodem12101
pio device monitor --port /dev/cu.usbmodem12101 -b 115200
```

The upload port changes; check `pio device list` (Espressif VID `303A`) rather than hardcoding it.
