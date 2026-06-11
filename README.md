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

