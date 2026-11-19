#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include "board_config.h"

#if defined(BOARD_FNK0104)
#include <TFT_eSPI.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include "imu.h"
#include "fluid_sim.h"
#include "wifi_ota.h"
TFT_eSPI display = TFT_eSPI();
#else
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_W, SCREEN_H, &SPI, OLED_CS, OLED_DC, OLED_RST);
#endif

static const int W = GRID_W;
static const int H = GRID_H;
static const int GRID_BYTES = (GRID_W * GRID_H + 7) / 8;

Adafruit_NeoPixel pixels(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Battery checking
unsigned long lastBatteryCheck = 0;
float filteredVbat = 0.0f;
static bool vbatInit = false;
uint8_t ledR = 0, ledG = 255, ledB = 0;   // default green

// FPS counter
unsigned long lastFPSTime = 0;
int frameCount = 0;

// Bit-packed grid. On the Freenove these live in PSRAM / internal heap.
#if defined(BOARD_FNK0104)
static uint8_t *grid = nullptr;
static uint8_t *nextGrid = nullptr;
static uint8_t *age = nullptr;
static uint8_t *historyBuf = nullptr;
static inline uint8_t *historyFrame(int i) { return historyBuf + i * GRID_BYTES; }
#else
uint8_t grid[GRID_BYTES];
uint8_t nextGrid[GRID_BYTES];
uint8_t age[GRID_W * GRID_H];
uint8_t history[6][GRID_BYTES];
static inline uint8_t *historyFrame(int i) { return history[i]; }
#endif

// Framebuffer is the physical screen (may be larger than the grid).
#if defined(BOARD_FNK0104)
static uint16_t *framebuffer = nullptr;
#else
static uint16_t framebuffer[SCREEN_W * SCREEN_H];
#endif

// End-state detection
static const int HISTORY = 6;
int historyIndex = 0;

// Global hue shift
uint16_t hueShift = 0;
static bool paused = false;

// Zoom viewport origin for 2x/4x modes (wraps around the toroidal grid).
static int viewX = 0;
static int viewY = 0;

// Styles
int currentStyle = 0;
const int NUM_STYLES = 6;

const char* styleNames[NUM_STYLES] = {
    "Classic White",
    "Rainbow Hue",
    "Heatmap",
    "Neon Trails",
    "Plasma",
    "Fire"
};

// Renders. On the Freenove, 1:1 is a 320x240 grid (one pixel per cell).
int currentRender = 0;
const int NUM_RENDERS = 3;

const char* renderNames[NUM_RENDERS] = {
#if defined(BOARD_FNK0104)
    "1:1 Render",
    "2x Zoom",
    "4x Zoom"
#else
    "1x Render",
    "2x Render",
    "4x Render"
#endif
};

#if defined(BOARD_FNK0104)
static Imu imu;
static FluidSim fluid;
static bool fluidMode = false;
static bool dmaReady = false;
static float touchGravityX = 0.0f;
static float touchGravityY = 1.0f;
static bool haveTouchGravity = false;
static bool fluidSteering = false;  // true while a finger is dragging in fluid mode
#endif

inline int idx(int x, int y) { return y * W + x; }
inline int fbIdx(int x, int y) { return y * SCREEN_W + x; }

static inline int wrapX(int x) {
    if (x < 0) return x + W;
    if (x >= W) return x - W;
    return x;
}
static inline int wrapY(int y) {
    if (y < 0) return y + H;
    if (y >= H) return y - H;
    return y;
}

inline bool getCell(uint8_t *buf, int x, int y) {
    int i = idx(x, y);
    return (buf[i >> 3] >> (i & 7)) & 1;
}

inline void setCell(uint8_t *buf, int x, int y, bool v) {
    int i = idx(x, y);
    uint8_t mask = 1 << (i & 7);
    if (v) buf[i >> 3] |= mask;
    else   buf[i >> 3] &= ~mask;
}

void randomizeGrid() {
    memset(grid, 0, GRID_BYTES);
    memset(nextGrid, 0, GRID_BYTES);
    memset(age, 0, GRID_W * GRID_H);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            bool alive = (random(100) < 25);
            setCell(grid, x, y, alive);
        }
    }
}

int countNeighbors(int x, int y) {
    int c = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = wrapX(x + dx);
            int ny = wrapY(y + dy);
            if (getCell(grid, nx, ny)) c++;
        }
    }
    return c;
}

static inline int neighbors8(int xm, int x, int xp, int ym, int y, int yp) {
    return (int)getCell(grid, xm, ym) + getCell(grid, x, ym) + getCell(grid, xp, ym)
         + (int)getCell(grid, xm, y)                       + getCell(grid, xp, y)
         + (int)getCell(grid, xm, yp) + getCell(grid, x, yp) + getCell(grid, xp, yp);
}

static inline void writeNext(int x, int y, int n) {
    const bool alive = getCell(grid, x, y);
    const bool nextAlive = alive ? (n == 2 || n == 3) : (n == 3);
    setCell(nextGrid, x, y, nextAlive);
    const int i = idx(x, y);
    if (nextAlive) age[i] = (uint8_t)min(255, (int)age[i] + 8);
    else           age[i] = (uint8_t)max(0, (int)age[i] - 3);
}

void stepLife() {
    // Interior cells skip wrap checks (modulo on every neighbor was killing 1:1 fps).
    for (int y = 1; y < H - 1; y++) {
        for (int x = 1; x < W - 1; x++) {
            writeNext(x, y, neighbors8(x - 1, x, x + 1, y - 1, y, y + 1));
        }
    }
    for (int x = 0; x < W; x++) {
        const int xm = wrapX(x - 1), xp = wrapX(x + 1);
        writeNext(x, 0,     neighbors8(xm, x, xp, H - 1, 0, 1));
        writeNext(x, H - 1, neighbors8(xm, x, xp, H - 2, H - 1, 0));
    }
    for (int y = 1; y < H - 1; y++) {
        const int ym = y - 1, yp = y + 1;
        writeNext(0,     y, neighbors8(W - 1, 0, 1, ym, y, yp));
        writeNext(W - 1, y, neighbors8(W - 2, W - 1, 0, ym, y, yp));
    }
    memcpy(grid, nextGrid, GRID_BYTES);
}

// HSV → RGB565
uint16_t hsvTo565(uint16_t h, uint8_t s, uint8_t v) {
    uint8_t region = h / 60;
    uint16_t remainder = (h - region * 60) * 255 / 60;

    uint8_t p = (v * (255 - s)) >> 8;
    uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    uint8_t r, g, b;
    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    return display.color565(r, g, b);
}

// Heatmap
uint16_t heatColor(uint8_t v) {
    uint8_t r, g, b;
    if (v < 64) {
        r = 0; g = v * 4; b = 255;
    } else if (v < 128) {
        r = 0; g = 255; b = 255 - (v - 64) * 4;
    } else if (v < 192) {
        r = (v - 128) * 4; g = 255; b = 0;
    } else {
        r = 255; g = 255 - (v - 192) * 4; b = 0;
    }
    return display.color565(r, g, b);
}

// Neon trails
uint16_t neonColor(uint8_t v) {
    uint8_t brightness = min(255, v * 2);   // double brightness
    return hsvTo565((v * 3) % 360, 255, brightness);
}

// Plasma
uint16_t plasmaColor(int x, int y) {
    float v =
        sinf(x * 0.12f) +
        sinf(y * 0.15f) +
        sinf((x + y) * 0.08f) +
        sinf(sqrtf(x*x + y*y) * 0.05f);

    // Normalize to 0–359 hue
    uint16_t h = (uint16_t)((v + 4.0f) * 45.0f) + hueShift;
    h %= 360;

    return hsvTo565(h, 255, 255);
}

// Fire
uint16_t fireColor(uint8_t v) {
    return display.color565(
        min(255, v * 2),
        min(255, v / 2),
        0
    );
}

static inline bool styleUsesAge() {
    // Styles that look best when using the "strongest" age in a block.
    return (currentStyle == 2) || (currentStyle == 3) || (currentStyle == 5);
}

static inline uint16_t styleColorAt(int x, int y, uint8_t ageVal) {
    switch (currentStyle) {
        case 0: // White
            return 0xFFFF;
        case 1: // Rainbow hue cycling
            return hsvTo565((hueShift + x + y) % 360, 255, 255);
        case 2: // Heatmap
            return heatColor(ageVal);
        case 3: // Neon trails
            return neonColor(ageVal);
        case 4: // Plasma
            return plasmaColor(x, y);
        case 5: // Fire
            return fireColor(ageVal);
        default:
            return 0xFFFF;
    }
}

static void fillCell(int dx, int dy, int size, uint16_t color) {
    for (int yy = 0; yy < size; yy++) {
        uint16_t *row = &framebuffer[fbIdx(dx, dy + yy)];
        for (int xx = 0; xx < size; xx++) {
            row[xx] = color;
        }
    }
}

static void waitForDma() {
#if defined(BOARD_FNK0104)
    if (dmaReady) display.dmaWait();
#endif
}

// 1x / 1:1 Render: each grid cell is CELL_SIZE x CELL_SIZE pixels and fills the screen.
void renderToFramebuffer() {
#if defined(BOARD_FNK0104)
    if (CELL_SIZE == 1) {
        for (int y = 0; y < H; y++) {
            uint16_t *row = &framebuffer[fbIdx(0, y)];
            for (int x = 0; x < W; x++) {
                const bool alive = getCell(grid, x, y);
                row[x] = alive ? styleColorAt(x, y, age[idx(x, y)]) : 0x0000;
            }
        }
        return;
    }
#endif
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const bool alive = getCell(grid, x, y);
            const uint16_t color = alive ? styleColorAt(x, y, age[idx(x, y)]) : 0x0000;
            fillCell(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, color);
        }
    }
}

// 2x Render: show a (W/2)x(H/2) viewport scaled to fill the screen.
void renderToFramebuffer2x() {
    const int zoom = 2;
    const int vw = W / zoom;
    const int vh = H / zoom;
    const int pix = CELL_SIZE * zoom;
    for (int sy = 0; sy < vh; sy++) {
        for (int sx = 0; sx < vw; sx++) {
            const int gx = wrapX(viewX + sx);
            const int gy = wrapY(viewY + sy);
            const bool alive = getCell(grid, gx, gy);
            const uint16_t color = alive ? styleColorAt(gx, gy, age[idx(gx, gy)]) : 0x0000;
            fillCell(sx * pix, sy * pix, pix, color);
        }
    }
}

// 4x Render: show a (W/4)x(H/4) viewport scaled to fill the screen.
void renderToFramebuffer4x() {
    const int zoom = 4;
    const int vw = W / zoom;
    const int vh = H / zoom;
    const int pix = CELL_SIZE * zoom;
    for (int sy = 0; sy < vh; sy++) {
        for (int sx = 0; sx < vw; sx++) {
            const int gx = wrapX(viewX + sx);
            const int gy = wrapY(viewY + sy);
            const bool alive = getCell(grid, gx, gy);
            const uint16_t color = alive ? styleColorAt(gx, gy, age[idx(gx, gy)]) : 0x0000;
            fillCell(sx * pix, sy * pix, pix, color);
        }
    }
}

void pushFramebufferDMA() {
#if defined(BOARD_FNK0104)
    if (dmaReady) {
        display.startWrite();
        display.pushImageDMA(0, 0, SCREEN_W, SCREEN_H, framebuffer);
        display.endWrite();
    } else {
        display.pushImage(0, 0, SCREEN_W, SCREEN_H, framebuffer);
    }
#else
    display.startWrite();
    display.setAddrWindow(0, 0, SCREEN_W, SCREEN_H);
    display.writePixels(framebuffer, SCREEN_W * SCREEN_H);
    display.endWrite();
#endif
}

bool gridsEqual(uint8_t *a, uint8_t *b) {
    return memcmp(a, b, GRID_BYTES) == 0;
}

bool isExtinct(uint8_t *g) {
    for (int i = 0; i < GRID_BYTES; i++)
        if (g[i] != 0) return false;
    return true;
}

void breatheLED(float speed = 0.03f, float maxBrightness = 0.15f) {
    static float t = 0.0f;

    // Base sine wave 0 → 1 → 0
    float wave = (sinf(t) + 1.0f) * 0.5f;

    // Smooth quadratic easing
    float shaped = wave * wave;

    // Apply brightness cap (0..1)
    float brightness = shaped * maxBrightness;

    // Scale colour chosen by battery logic
    uint8_t R = ledR * brightness;
    uint8_t G = ledG * brightness;
    uint8_t B = ledB * brightness;

    pixels.setPixelColor(0, pixels.Color(R, G, B));
    pixels.show();

    t += speed;
    if (t > 2 * PI) t -= 2 * PI;
}

// Call breatheLED at most ~20 Hz; NeoPixel updates are not free.
static void maybeBreatheLED(float speed, float maxBrightness) {
    static unsigned long last = 0;
    unsigned long now = millis();
    if (now - last < 50) return;
    last = now;
    breatheLED(speed, maxBrightness);
}

int lipoPercent(float v) {
    if (v >= 4.20f) return 100;
    if (v >= 4.15f) return 95;
    if (v >= 4.10f) return 90;
    if (v >= 4.05f) return 85;
    if (v >= 4.00f) return 80;
    if (v >= 3.95f) return 70;
    if (v >= 3.90f) return 60;
    if (v >= 3.85f) return 50;
    if (v >= 3.80f) return 40;
    if (v >= 3.75f) return 30;
    if (v >= 3.70f) return 20;
    if (v >= 3.60f) return 10;
    if (v >= 3.50f) return 5;
    if (v >= 3.40f) return 2;
    return 0;   // ~3.30–3.35V under load
}

void setBatteryLED(float vbat) {
    int pct = lipoPercent(vbat);

    if (pct >= 85)      { ledR = 0;   ledG = 255; ledB = 0;   }   // green
    else if (pct >= 70) { ledR = 80;  ledG = 255; ledB = 0;   }   // yellow-green
    else if (pct >= 50) { ledR = 255; ledG = 255; ledB = 0;   }   // yellow
    else if (pct >= 30) { ledR = 255; ledG = 180; ledB = 0;   }   // orange-yellow
    else if (pct >= 15) { ledR = 255; ledG = 100; ledB = 0;   }   // orange
    else if (pct >= 5)  { ledR = 255; ledG = 40;  ledB = 0;   }   // red-orange
    else                { ledR = 255; ledG = 0;   ledB = 0;   }   // red
}

void updateBattery() {
    unsigned long now = millis();
    if (now - lastBatteryCheck < 10000) return;   // run every 10s
    lastBatteryCheck = now;

    const int samples = 16;
    float sumMv = 0;

    for (int i = 0; i < samples; i++) {
        sumMv += analogReadMilliVolts(BAT_ADC_PIN);
        delay(2);   // tiny delay improves stability
    }

    float rawAvgMv = sumMv / (float)samples;

#if defined(BAT_DIVIDER)
    float vbat = (rawAvgMv * BAT_DIVIDER) / 1000.0f;
#elif defined(BAT_SCALE)
    float vbat = (rawAvgMv / 1000.0f) * BAT_SCALE;
#else
    float vbat = rawAvgMv / 1000.0f;
#endif

    if (!vbatInit) {
        filteredVbat = vbat;
        vbatInit = true;
    } else {
        filteredVbat = (filteredVbat * 0.85f) + (vbat * 0.15f);
    }

    setBatteryLED(filteredVbat);
}

bool isCharging() {
#if defined(CHARGE_SENSE_PIN)
    return digitalRead(CHARGE_SENSE_PIN) == HIGH;   // 5V present
#else
    // FNK0104B has no dedicated 5V-sense GPIO in the official pinout.
    return false;
#endif
}

void updateFPS() {
    frameCount++;

    unsigned long now = millis();
    if (now - lastFPSTime >= 1000) {   // 1 second
        Serial.print("FPS: ");
        Serial.println(frameCount);

        frameCount = 0;
        lastFPSTime = now;
    }
}

static void clearHistoryFrames() {
    for (int i = 0; i < HISTORY; i++) {
        memset(historyFrame(i), 0, GRID_BYTES);
    }
    historyIndex = 0;
}

#if defined(BOARD_FNK0104)
static void toggleFluidMode() {
    fluidMode = !fluidMode;
    if (fluidMode) {
        if (!fluid.ready()) fluid.begin();
        else fluid.reseed();
        Serial.println("Mode: FLUID");
    } else {
        Serial.println("Mode: GoL");
    }
}
#endif

static void printControls() {
    Serial.println();
#if defined(BOARD_FNK0104)
    Serial.println("GoL / fluid controls:");
    Serial.println("  r = next render (1:1, 2x zoom, 4x zoom)");
    Serial.println("  e = next style");
    Serial.println("  n = reseed grid (or fluid pool)");
    Serial.println("  p = pause/resume");
    Serial.println("  f = toggle GoL / fluid");
    Serial.println("  i = I2C scan + IMU status");
    Serial.println("  m = cycle IMU axis mapping");
    Serial.println("  w/a/s/d = pan viewport (2x/4x)");
    Serial.println("  h/? = help");
    Serial.println("  Touch: tap = reseed (GoL) or stir (fluid)");
    Serial.println("         long-press = pause");
    Serial.println("         two-finger tap = toggle GoL / fluid");
    Serial.println("         long-press a corner = toggle GoL / fluid");
    Serial.println("         1:1 swipe L/R = style, U/D = render");
    Serial.println("         2x/4x swipe = pan");
    Serial.println("         in fluid, drag sets gravity (also the no-IMU fallback)");
    Serial.println("  BOOT: short = next style (GoL) or stir (fluid)");
    Serial.println("        double-tap = toggle GoL / fluid");
    Serial.println("        long = next render (GoL)");
#else
    Serial.println("GoL controls:");
    Serial.println("  r = next render");
    Serial.println("  e = next style");
    Serial.println("  n = reseed grid");
    Serial.println("  p = pause/resume");
    Serial.println("  w/a/s/d = pan viewport (2x/4x)");
    Serial.println("  h/? = help");
#endif
    Serial.println();
}

static void handleSerialControls() {
    static bool printedOnFirstInput = false;
    while (Serial.available() > 0) {
        if (!printedOnFirstInput) {
            // If the serial monitor attaches after boot, the user may miss the boot-time help.
            printControls();
            printedOnFirstInput = true;
        }

        const char c = (char)Serial.read();
        switch (c) {
            case 'r':
            case 'R':
                currentRender = (currentRender + 1) % NUM_RENDERS;
                Serial.print("Render: ");
                Serial.println(renderNames[currentRender]);
                break;
            case 'e':
            case 'E':
                currentStyle = (currentStyle + 1) % NUM_STYLES;
                Serial.print("Style: ");
                Serial.println(styleNames[currentStyle]);
                break;
            case 'n':
            case 'N':
#if defined(BOARD_FNK0104)
                if (fluidMode) {
                    fluid.reseed();
                    Serial.println("Reseeded fluid.");
                    break;
                }
#endif
                randomizeGrid();
                clearHistoryFrames();
                Serial.println("Reseeded grid.");
                break;
            case 'p':
            case 'P':
                paused = !paused;
                Serial.print("Paused: ");
                Serial.println(paused ? "yes" : "no");
                break;
#if defined(BOARD_FNK0104)
            case 'f':
            case 'F':
                toggleFluidMode();
                break;
            case 'i':
            case 'I':
                imu.scanBus();
                if (imu.ok()) {
                    Serial.printf("IMU %s at 0x%02X\n", imu.kindName(), imu.address());
                    imu.printMapping();
                    float gx, gy;
                    bool ok = imu.read(gx, gy);
                    Serial.printf("  gravity sample: gx=%.2f gy=%.2f %s\n",
                                  gx, gy, ok ? "ok" : "fallback");
                } else {
                    Serial.println("IMU not found; fluid uses touch-drag gravity.");
                }
                break;
            case 'm':
            case 'M':
                imu.cycleMapping();
                break;
#endif
            case 'w':
            case 'W':
                viewY = wrapY(viewY - 1);
                break;
            case 's':
            case 'S':
                viewY = wrapY(viewY + 1);
                break;
            case 'a':
            case 'A':
                viewX = wrapX(viewX - 1);
                break;
            case 'd':
            case 'D':
                viewX = wrapX(viewX + 1);
                break;
            case 'h':
            case 'H':
            case '?':
                printControls();
                break;
            default:
                break;
        }
    }
}

#if defined(HAS_TOUCH)
static bool touchReady = false;

struct TouchPoint {
    int x, y;
};

static int readTouches(TouchPoint pts[2]) {
    if (!touchReady) return 0;

    // Arduino-ESP32 3.x: avoid repeated-start (endTransmission(false)),
    // which returns ESP_ERR_INVALID_STATE on this I2C driver.
    Wire.beginTransmission(TOUCH_I2C_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission() != 0) return 0;

    uint8_t buf[11];
    if (Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)11) != 11) return 0;
    for (int i = 0; i < 11; i++) buf[i] = Wire.read();

    int n = buf[0] & 0x0F;
    if (n <= 0) return 0;
    if (n > 2) n = 2;

    for (int i = 0; i < n; i++) {
        // Point 1 at +1, point 2 at +7 (skip weight/misc).
        const int off = (i == 0) ? 1 : 7;
        int rawx = ((buf[off] & 0x0F) << 8) | buf[off + 1];
        int rawy = ((buf[off + 2] & 0x0F) << 8) | buf[off + 3];
        // Native panel is 240x320. Rotation 1 (landscape 320x240):
        // x' = y, y' = (native_width - 1) - x
        int x = rawy;
        int y = 239 - rawx;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x >= SCREEN_W) x = SCREEN_W - 1;
        if (y >= SCREEN_H) y = SCREEN_H - 1;
        pts[i].x = x;
        pts[i].y = y;
    }
    return n;
}

static void initTouch() {
    // Match Freenove's FT6336U::begin(): I2C first, then a 10ms reset
    // and a 500ms wait for the controller to come out of boot.
    pinMode(TOUCH_INT, INPUT);
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Wire.setClock(100000);

    pinMode(TOUCH_RST, OUTPUT);
    digitalWrite(TOUCH_RST, LOW);
    delay(10);
    digitalWrite(TOUCH_RST, HIGH);
    delay(500);

    Wire.beginTransmission(TOUCH_I2C_ADDR);
    touchReady = (Wire.endTransmission() == 0);
    Serial.print("FT6336 at 0x38: ");
    Serial.println(touchReady ? "ok" : "not found");
    if (touchReady) {
        Wire.setClock(400000);
    }
}

static bool inCorner(int x, int y) {
    const int m = 50;
    return (x < m || x >= SCREEN_W - m) && (y < m || y >= SCREEN_H - m);
}

static void handleTouch() {
    TouchPoint pts[2];
    const int n = readTouches(pts);
    const bool down = n > 0;
    static bool touchDown = false;
    static int touchStartX = 0, touchStartY = 0;
    static unsigned long touchStartMs = 0;
    static bool touchDidGesture = false;
    static bool twoFinger = false;
    static int twoStartX0 = 0, twoStartY0 = 0, twoStartX1 = 0, twoStartY1 = 0;

    if (n >= 2) {
        if (!twoFinger) {
            twoFinger = true;
            twoStartX0 = pts[0].x; twoStartY0 = pts[0].y;
            twoStartX1 = pts[1].x; twoStartY1 = pts[1].y;
            touchStartMs = millis();
            touchDidGesture = false;
        }
        touchDown = true;
        return;
    }

    if (down && !touchDown) {
        touchDown = true;
        twoFinger = false;
        touchStartX = pts[0].x;
        touchStartY = pts[0].y;
        touchStartMs = millis();
        touchDidGesture = false;
    } else if (down && touchDown && !twoFinger) {
        const int dx = pts[0].x - touchStartX;
        const int dy = pts[0].y - touchStartY;
#if defined(BOARD_FNK0104)
        if (fluidMode) {
            // Drag / press position sets gravity (also the no-IMU fallback).
            const float cx = (pts[0].x - SCREEN_W * 0.5f) / (SCREEN_W * 0.5f);
            const float cy = (pts[0].y - SCREEN_H * 0.5f) / (SCREEN_H * 0.5f);
            touchGravityX = cx;
            touchGravityY = cy;
            haveTouchGravity = true;
            fluidSteering = true;
            touchDidGesture = true;
            return;
        }
#endif
        if (!touchDidGesture && (abs(dx) > 30 || abs(dy) > 30)) {
            if (currentRender == 0) {
                touchDidGesture = true;
                if (abs(dx) > abs(dy)) {
                    if (dx > 0) currentStyle = (currentStyle + 1) % NUM_STYLES;
                    else        currentStyle = (currentStyle + NUM_STYLES - 1) % NUM_STYLES;
                    Serial.print("Style: ");
                    Serial.println(styleNames[currentStyle]);
                } else {
                    if (dy > 0) currentRender = (currentRender + 1) % NUM_RENDERS;
                    else        currentRender = (currentRender + NUM_RENDERS - 1) % NUM_RENDERS;
                    Serial.print("Render: ");
                    Serial.println(renderNames[currentRender]);
                }
            } else {
                const int step = (currentRender == 1) ? 4 : 8;
                if (abs(dx) > abs(dy)) {
                    viewX = wrapX(viewX + (dx > 0 ? -step : step));
                } else {
                    viewY = wrapY(viewY + (dy > 0 ? -step : step));
                }
                touchStartX = pts[0].x;
                touchStartY = pts[0].y;
            }
        }
    } else if (!down && touchDown) {
        if (twoFinger) {
            const unsigned long held = millis() - touchStartMs;
            // A short two-finger tap toggles modes. Ignore if the fingers slid a lot.
            const int move = abs(pts[0].x - twoStartX0) + abs(pts[0].y - twoStartY0);
            (void)move;
            if (held < 800) {
#if defined(BOARD_FNK0104)
                toggleFluidMode();
#endif
            }
        } else if (!touchDidGesture) {
            const bool corner = inCorner(touchStartX, touchStartY);
            if (millis() - touchStartMs >= 400) {
#if defined(BOARD_FNK0104)
                if (corner) {
                    toggleFluidMode();
                } else
#endif
                {
                    paused = !paused;
                    Serial.print("Paused: ");
                    Serial.println(paused ? "yes" : "no");
                }
            } else {
#if defined(BOARD_FNK0104)
                if (fluidMode) {
                    fluid.stir();
                    Serial.println("Stirred fluid.");
                } else
#endif
                {
                    randomizeGrid();
                    clearHistoryFrames();
                    Serial.println("Reseeded grid.");
                }
            }
        }
        touchDown = false;
        twoFinger = false;
        fluidSteering = false;
    }
}

static void handleBootButton() {
    static bool lastUp = true;
    static unsigned long pressStart = 0;
    static unsigned long lastShortMs = 0;
    static bool longFired = false;
    static bool pendingShort = false;
    static bool secondTap = false;
    const bool up = digitalRead(BOOT_BTN_PIN);   // HIGH = not pressed
    const bool pressed = !up;
    const unsigned long now = millis();
    const unsigned long kDoubleMs = 300;
    const unsigned long kLongMs = 500;

    if (pressed && lastUp) {
        pressStart = now;
        longFired = false;
        if (pendingShort && (now - lastShortMs) <= kDoubleMs) {
            secondTap = true;
            pendingShort = false;
        } else {
            secondTap = false;
        }
    } else if (pressed && !longFired && (now - pressStart >= kLongMs)) {
        longFired = true;
        pendingShort = false;
        secondTap = false;
#if defined(BOARD_FNK0104)
        if (!fluidMode)
#endif
        {
            currentRender = (currentRender + 1) % NUM_RENDERS;
            Serial.print("Render: ");
            Serial.println(renderNames[currentRender]);
        }
    } else if (!pressed && !lastUp && !longFired) {
        if (secondTap) {
            secondTap = false;
            pendingShort = false;
#if defined(BOARD_FNK0104)
            toggleFluidMode();
#endif
        } else {
            pendingShort = true;
            lastShortMs = now;
        }
    }
    lastUp = up;

    if (pendingShort && !pressed && (now - lastShortMs >= kDoubleMs)) {
        pendingShort = false;
#if defined(BOARD_FNK0104)
        if (fluidMode) {
            fluid.stir();
            Serial.println("Stirred fluid.");
        } else
#endif
        {
            currentStyle = (currentStyle + 1) % NUM_STYLES;
            Serial.print("Style: ");
            Serial.println(styleNames[currentStyle]);
        }
    }
}
#endif

static bool allocBuffers() {
#if defined(BOARD_FNK0104)
    auto take = [](size_t n, bool preferInternal) -> void * {
        void *p = nullptr;
        if (preferInternal) {
            p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (!p) {
            p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (!p) p = malloc(n);
        return p;
    };

    framebuffer = (uint16_t *)take(SCREEN_W * SCREEN_H * sizeof(uint16_t), false);
    grid = (uint8_t *)take(GRID_BYTES, true);
    nextGrid = (uint8_t *)take(GRID_BYTES, true);
    age = (uint8_t *)take(GRID_W * GRID_H, true);
    historyBuf = (uint8_t *)take(HISTORY * GRID_BYTES, false);
    if (!framebuffer || !grid || !nextGrid || !age || !historyBuf) {
        Serial.println("Buffer alloc failed");
        return false;
    }
    memset(grid, 0, GRID_BYTES);
    memset(nextGrid, 0, GRID_BYTES);
    memset(age, 0, GRID_W * GRID_H);
    Serial.printf("Buffers ok  FB=%s  free internal=%u  psram=%u\n",
                  heap_caps_get_allocated_size(framebuffer) ? "yes" : "yes",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#endif
    return true;
}

static void initDisplay() {
#if defined(BOARD_FNK0104)
    display.init();
    display.setRotation(1);          // landscape 320x240
    display.setSwapBytes(true);
    dmaReady = display.initDMA();
    Serial.printf("TFT DMA: %s\n", dmaReady ? "on" : "off, using pushImage");
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextDatum(MC_DATUM);
    display.drawString("Game of Life", SCREEN_W / 2, SCREEN_H / 2, 4);
#else
    display.begin();
    display.fillScreen(0);
    display.setTextColor(0xFFFF);
    display.setTextSize(1);
    display.setCursor(20, 60);
    display.print("Game of Life");
#endif
}

void setup() {
    Serial.begin(115200);
    delay(200);

    if (!allocBuffers()) {
        while (true) delay(1000);
    }

    initDisplay();

#if defined(CHARGE_SENSE_PIN)
    pinMode(CHARGE_SENSE_PIN, INPUT);   // 5V sense
#endif

#if defined(LDO2_EN)
    pinMode(LDO2_EN, OUTPUT);
    digitalWrite(LDO2_EN, HIGH);   // enable LDO2 (powers OLED)
#endif

    pixels.begin();
    pixels.show();

    analogReadResolution(12);
#if !defined(BOARD_FNK0104)
    // pinMode(INPUT) on an ADC pin can mark it digital on Arduino-ESP32 3.x.
    pinMode(BAT_ADC_PIN, INPUT);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
#endif

#if defined(HAS_TOUCH)
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
    initTouch();
#endif

#if defined(BOARD_FNK0104)
    imu.begin();
    fluid.begin();
    wifiOtaBegin();
#endif

    clearHistoryFrames();
#if !defined(BOARD_FNK0104)
    memset(age, 0, sizeof(age));
#endif

    randomSeed(esp_random());
    randomizeGrid();
    currentStyle = random(0, NUM_STYLES);
#if defined(BOARD_FNK0104)
    currentRender = 0;  // boot into 1:1 so the full 320x240 grid is visible
#else
    currentRender = random(0, NUM_RENDERS);
#endif
    delay(400);
    Serial.println("GoL boot");
#if defined(BOARD_FNK0104)
    Serial.println("Board: Freenove FNK0104B 2.8in ILI9341");
    Serial.printf("Grid %dx%d, cell %dpx, screen %dx%d\n",
                  GRID_W, GRID_H, CELL_SIZE, SCREEN_W, SCREEN_H);
#endif
    Serial.print("Selected style: ");
    Serial.println(styleNames[currentStyle]);
    Serial.print("Selected render: ");
    Serial.println(renderNames[currentRender]);
    printControls();
}

void loop() {
#if defined(BOARD_FNK0104)
    wifiOtaPoll();
#endif
    handleSerialControls();
#if defined(HAS_TOUCH)
    if (touchReady) handleTouch();
    handleBootButton();
#endif

#if defined(BOARD_FNK0104)
    if (fluidMode) {
        if (!paused) {
            float gx = 0.0f, gy = 1.0f;
            const bool gotImu = imu.read(gx, gy);
            // Live touch-drag overrides the IMU so you can steer even if the
            // axis map is wrong. Without an IMU, the last drag (or down) is used.
            if (fluidSteering || !gotImu) {
                gx = haveTouchGravity ? touchGravityX : 0.0f;
                gy = haveTouchGravity ? touchGravityY : 1.0f;
            }
            fluid.step(gx, gy, 1.0f / 45.0f);
        }
        waitForDma();
        fluid.render(framebuffer, SCREEN_W, SCREEN_H);
        pushFramebufferDMA();
        updateFPS();
        updateBattery();
        float speed = isCharging() ? 0.70f : 0.02f;
        maybeBreatheLED(speed, 0.10f);
        return;
    }
#endif

    if (!paused) {
        hueShift = (hueShift + 1) % 360;
        stepLife();

        // --- Multi-frame cycle detection ---
        bool cycleDetected = false;

        // Compare current grid against all history frames
        for (int i = 0; i < HISTORY; i++) {
            if (memcmp(grid, historyFrame(i), GRID_BYTES) == 0) {
                cycleDetected = true;
                break;
            }
        }

        if (isExtinct(grid) || cycleDetected) {
            randomizeGrid();
            clearHistoryFrames();
            currentStyle = random(0, NUM_STYLES);
#if !defined(BOARD_FNK0104)
            currentRender = random(0, NUM_RENDERS);
#endif

            Serial.print("Restart triggered — new style: ");
            Serial.println(styleNames[currentStyle]);
            Serial.print("New render: ");
            Serial.println(renderNames[currentRender]);
        }

        // Store current grid into history
        memcpy(historyFrame(historyIndex), grid, GRID_BYTES);
        historyIndex = (historyIndex + 1) % HISTORY;
    }

    waitForDma();
    switch (currentRender) {
        case 0: renderToFramebuffer(); break;
        case 1: renderToFramebuffer2x(); break;
        case 2: renderToFramebuffer4x(); break;
    }

    pushFramebufferDMA();

#if defined(BOARD_FNK0104)
    updateFPS();
#endif

    updateBattery();

    float speed = isCharging() ? 0.70f : 0.02f; // breathing speeds up when charging - higher = faster
    maybeBreatheLED(speed, 0.10f);   // 10% brightness cap
}
