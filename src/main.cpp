#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <Adafruit_NeoPixel.h>

#define OLED_CS   5
#define OLED_DC   4
#define OLED_RST  2

Adafruit_SSD1351 display = Adafruit_SSD1351(128, 128, &SPI, OLED_CS, OLED_DC, OLED_RST);

static const int W = 128;
static const int H = 128;

Adafruit_NeoPixel pixels(1, 18, NEO_GRB + NEO_KHZ800);

// Battery checking
unsigned long lastBatteryCheck = 0;
float filteredVbat = 0.0f;
static bool vbatInit = false;
uint8_t ledR = 0, ledG = 255, ledB = 0;   // default green

// FPS counter
unsigned long lastFPSTime = 0;
int frameCount = 0;

// Bit-packed grid
uint8_t grid[(W * H) / 8];
uint8_t nextGrid[(W * H) / 8];

// Framebuffer
uint16_t framebuffer[W * H];

// End-state detection
static const int HISTORY = 6;
uint8_t history[HISTORY][(W * H) / 8];
int historyIndex = 0;

// Age buffer (for heatmap / neon / fire)
uint8_t age[W * H];

// Global hue shift
uint16_t hueShift = 0;

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

// Renders
int currentRender = 0;
const int NUM_RENDERS = 3;

const char* renderNames[NUM_RENDERS] = {
    "1x Render",
    "2x Render",
    "4x Render"
};

inline int idx(int x, int y) { return y * W + x; }

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
    memset(grid, 0, sizeof(grid));
    memset(nextGrid, 0, sizeof(nextGrid));
    memset(age, 0, sizeof(age));

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
            int nx = (x + dx + W) % W;
            int ny = (y + dy + H) % H;
            if (getCell(grid, nx, ny)) c++;
        }
    }
    return c;
}

void stepLife() {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            bool alive = getCell(grid, x, y);
            int n = countNeighbors(x, y);
            bool nextAlive = alive ? (n == 2 || n == 3) : (n == 3);
            setCell(nextGrid, x, y, nextAlive);

            int i = idx(x, y);
            if (nextAlive) age[i] = min<uint8_t>(255, age[i] + 8);
            else           age[i] = max<uint8_t>(0, age[i] - 3);
        }
    }
    memcpy(grid, nextGrid, sizeof(grid));
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

// 1x Render
void renderToFramebuffer() {
    hueShift = (hueShift + 1) % 360;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            bool alive = getCell(grid, x, y);
            int i = idx(x, y);

            if (!alive) {
                framebuffer[i] = 0x0000;
                continue;
            }

            switch (currentStyle) {
                case 0: // White
                    framebuffer[i] = 0xFFFF;
                    break;
                case 1: // Rainbow hue cycling
                    framebuffer[i] = hsvTo565((hueShift + x + y) % 360, 255, 255);
                    break;
                case 2: // Heatmap
                    framebuffer[i] = heatColor(age[i]);
                    break;
                case 3: // Neon trails
                    framebuffer[i] = neonColor(age[i]);
                    break;
                case 4: // Plasma
                    framebuffer[i] = plasmaColor(x, y);
                    break;
                case 5: // Fire
                    framebuffer[i] = fireColor(age[i]);
                    break;
                default:
                    framebuffer[i] = 0xFFFF; // fallback
                    break;
            }
        }
    }
}

// 2x Render
void renderToFramebuffer2x() {
    hueShift = (hueShift + 1) % 360;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {

            bool alive = getCell(grid, x, y);
            uint16_t color;

            if (!alive) {
                color = 0x0000;
            } else {
                int i = idx(x, y);
                switch (currentStyle) {
                    case 0: color = 0xFFFF; break; // White
                    case 1: color = hsvTo565((hueShift + x + y) % 360, 255, 255); break;
                    case 2: color = heatColor(age[i]); break;
                    case 3: color = neonColor(age[i]); break;
                    case 4: color = plasmaColor(x, y); break;
                    case 5: color = fireColor(age[i]); break;
                    default: color = 0xFFFF; break;
                }
            }

            // --- 2×2 pixel scaling ---
            int sx = x * 2;
            int sy = y * 2;

            if (sx < W-1 && sy < H-1) {
                framebuffer[idx(sx,     sy    )] = color;
                framebuffer[idx(sx + 1, sy    )] = color;
                framebuffer[idx(sx,     sy + 1)] = color;
                framebuffer[idx(sx + 1, sy + 1)] = color;
            }
        }
    }
}


// Alternate 2x Render
void renderToFramebuffer2xAlt() {
    hueShift = (hueShift + 1) % 360;

    // We render in 2×2 blocks, so we step by 2
    for (int y = 0; y < H; y += 2) {
        for (int x = 0; x < W; x += 2) {

            // Determine block state from 4 simulation cells
            bool alive =
                getCell(grid, x,     y    ) ||
                getCell(grid, x + 1, y    ) ||
                getCell(grid, x,     y + 1) ||
                getCell(grid, x + 1, y + 1);

            uint16_t color;

            if (!alive) {
                color = 0x0000; // black
            } else {
                // Use the top-left cell for age-based styles
                int i = idx(x, y);

                switch (currentStyle) {
                    case 0: // White
                        color = 0xFFFF;
                        break;

                    case 1: // Rainbow hue cycling
                        color = hsvTo565((hueShift + x + y) % 360, 255, 255);
                        break;

                    case 2: // Heatmap
                        color = heatColor(age[i]);
                        break;

                    case 3: // Neon trails
                        color = neonColor(age[i]);
                        break;

                    case 4: // Plasma
                        color = plasmaColor(x, y);
                        break;

                    case 5: // Fire
                        color = fireColor(age[i]);
                        break;

                    default:
                        color = 0xFFFF;
                        break;
                }
            }

            // Draw the 2×2 block
            framebuffer[idx(x,     y    )] = color;
            framebuffer[idx(x + 1, y    )] = color;
            framebuffer[idx(x,     y + 1)] = color;
            framebuffer[idx(x + 1, y + 1)] = color;
        }
    }
}


// 4x Render
void renderToFramebuffer4x() {
    hueShift = (hueShift + 1) % 360;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {

            bool alive = getCell(grid, x, y);
            uint16_t color;

            if (!alive) {
                color = 0x0000;
            } else {
                int i = idx(x, y);
                switch (currentStyle) {
                    case 0: color = 0xFFFF; break; // White
                    case 1: color = hsvTo565((hueShift + x + y) % 360, 255, 255); break;
                    case 2: color = heatColor(age[i]); break;
                    case 3: color = neonColor(age[i]); break;
                    case 4: color = plasmaColor(x, y); break;
                    case 5: color = fireColor(age[i]); break;
                    default: color = 0xFFFF; break;
                }
            }

            // --- 4×4 pixel scaling ---
            int sx = x * 4;
            int sy = y * 4;

            if (sx < W-3 && sy < H-3) {
                framebuffer[idx(sx,     sy    )] = color;
                framebuffer[idx(sx + 1, sy    )] = color;
                framebuffer[idx(sx + 2, sy    )] = color;
                framebuffer[idx(sx + 3, sy    )] = color;

                framebuffer[idx(sx,     sy + 1)] = color;
                framebuffer[idx(sx + 1, sy + 1)] = color;
                framebuffer[idx(sx + 2, sy + 1)] = color;
                framebuffer[idx(sx + 3, sy + 1)] = color;

                framebuffer[idx(sx,     sy + 2)] = color;
                framebuffer[idx(sx + 1, sy + 2)] = color;
                framebuffer[idx(sx + 2, sy + 2)] = color;
                framebuffer[idx(sx + 3, sy + 2)] = color;

                framebuffer[idx(sx,     sy + 3)] = color;
                framebuffer[idx(sx + 1, sy + 3)] = color;
                framebuffer[idx(sx + 2, sy + 3)] = color;
                framebuffer[idx(sx + 3, sy + 3)] = color;
            }
        }
    }
}


void pushFramebufferDMA() {
    display.startWrite();
    display.setAddrWindow(0, 0, W, H);
    display.writePixels(framebuffer, W * H);
    display.endWrite();
}

bool gridsEqual(uint8_t *a, uint8_t *b) {
    return memcmp(a, b, sizeof(grid)) == 0;
}

bool isExtinct(uint8_t *g) {
    for (int i = 0; i < (int)sizeof(grid); i++)
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

    // --- Take multiple ADC samples ---
    const int samples = 16;
    uint32_t sum = 0;

    for (int i = 0; i < samples; i++) {
        sum += analogRead(10);
        delay(2);   // tiny delay improves stability
    }

    float rawAvg = sum / (float)samples;

    // --- Convert to battery voltage (calibrated factor ≈ 3.95) ---
    float vbat = (rawAvg / 4095.0f) * 3.3f * 3.95f;

    // --- Filter (initialise on first run) ---
    if (!vbatInit) {
        filteredVbat = vbat;
        vbatInit = true;
    } else {
        filteredVbat = (filteredVbat * 0.85f) + (vbat * 0.15f);
    }

    // --- Debug ---
    //Serial.print("RAW AVG: ");
    //Serial.println(rawAvg);
    //Serial.print("Battery Voltage: ");
    //Serial.println(filteredVbat, 3);

    // --- LED update ---
    setBatteryLED(filteredVbat);
}

bool isCharging() {
    return digitalRead(33) == HIGH;   // 5V present
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

void setup() {
    Serial.begin(115200);
    display.begin();
    display.fillScreen(0);

    pinMode(33, INPUT);   // 5v sense

    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH);   // enable LDO2 (powers NeoPixel)

    pixels.begin();
    pixels.show();

    pinMode(10, INPUT);                       // vbat
    analogReadResolution(12);                 // 0–4095
    analogSetPinAttenuation(10, ADC_11db);    // allow up to ~3.3V input

    for (int i = 0; i < HISTORY; i++) {
        memset(history[i], 0, sizeof(history[i]));
    }
    memset(age, 0, sizeof(age));

    randomSeed(esp_random());
    randomizeGrid();
    currentStyle = random(0, NUM_STYLES);
    currentRender = random(0, NUM_RENDERS);
    sleep(1);
    Serial.print("Selected style: ");
    Serial.println(styleNames[currentStyle]);
    Serial.print("Selected render: ");
    Serial.println(renderNames[currentRender]);
}

void loop() {
    stepLife();

    // --- Multi-frame cycle detection ---
    bool cycleDetected = false;

    // Compare current grid against all history frames
    for (int i = 0; i < HISTORY; i++) {
        if (memcmp(grid, history[i], sizeof(grid)) == 0) {
            cycleDetected = true;
            break;
        }
    }

    if (isExtinct(grid) || cycleDetected) {
        randomizeGrid();
        currentStyle = random(0, NUM_STYLES);
        currentRender = random(0, NUM_RENDERS);

        Serial.print("Restart triggered — new style: ");
        Serial.println(styleNames[currentStyle]);
        Serial.print("New render: ");
        Serial.println(renderNames[currentRender]);
    }

    // Store current grid into history
    memcpy(history[historyIndex], grid, sizeof(grid));
    historyIndex = (historyIndex + 1) % HISTORY;

    switch (currentRender) {
        case 0: renderToFramebuffer(); break;
        case 1: renderToFramebuffer2x(); break;
        case 2: renderToFramebuffer4x(); break;
    }    

    pushFramebufferDMA();

    //updateFPS();
    
    updateBattery();

    float speed = isCharging() ? 0.70f : 0.02f; // breathing speeds up when charging - higher = faster
    breatheLED(speed, 0.10f);   // 10% brightness cap


}
