#include "board_config.h"

#if defined(BOARD_FNK0104)

#include "imu.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// Stop-then-start. Repeated starts (endTransmission(false)) return
// ESP_ERR_INVALID_STATE on Arduino-ESP32 3.x's I2C driver.
bool readRegs(uint8_t addr, uint8_t reg, uint8_t* dest, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(addr, len) != len) return false;
    for (uint8_t i = 0; i < len; ++i) dest[i] = Wire.read();
    return true;
}

bool i2cPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

const char* identify(uint8_t addr, uint8_t& who, ImuKind& kind) {
    who = 0;
    kind = IMU_UNKNOWN;
    uint8_t id = 0;
    if (addr == TOUCH_I2C_ADDR) {
        kind = IMU_NONE;
        return "FT6336 touch";
    }
    if (readRegs(addr, 0x75, &id, 1)) {
        if (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72 || id == 0x73 ||
            id == 0x12 || id == 0x11 || id == 0x98) {
            who = id;
            kind = IMU_MPU6050;
            if (id == 0x68) return "MPU6050";
            if (id == 0x70) return "MPU6500";
            if (id == 0x71) return "MPU9250";
            return "MPU/ICM (0x75)";
        }
    }
    if (readRegs(addr, 0x0F, &id, 1)) {
        if (id == 0x69 || id == 0x6A || id == 0x6B || id == 0x6C) {
            who = id;
            kind = IMU_LSM6;
            return "LSM6";
        }
        if (id == 0x33) {
            who = id;
            kind = IMU_LIS3DH;
            return "LIS3DH";
        }
    }
    if (readRegs(addr, 0x00, &id, 1)) {
        if (id == 0x05) {
            who = id;
            kind = IMU_QMI8658;
            return "QMI8658";
        }
        if (id == 0xD1) {
            who = id;
            kind = IMU_BMI160;
            return "BMI160";
        }
        if (id == 0xEA) {
            who = id;
            kind = IMU_UNKNOWN;
            return "ICM-20948?";
        }
        who = id;
    }
    return nullptr;
}

int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }
int16_t le16(const uint8_t* p) { return (int16_t)((p[1] << 8) | p[0]); }

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// axis: 0=X 1=Y 2=Z, sign ±1. Twelve common board orientations.
// Map 0 is the default: screenX=+X, screenY=-Y (flipped Y; "flip y").
const int kMaps[][4] = {
    {0, 1,  1, -1},
    {0, 1, -1,  1},
    {0, 1,  1,  1},
    {0, 1, -1, -1},
    {1, 0,  1,  1},
    {1, 0, -1,  1},
    {1, 0,  1, -1},
    {1, 0, -1, -1},
    {0, 2,  1,  1},
    {0, 2, -1,  1},
    {0, 2,  1, -1},
    {0, 2, -1, -1},
};
const int kMapCount = 12;

float pick(float x, float y, float z, int axis) {
    if (axis == 0) return x;
    if (axis == 1) return y;
    return z;
}

}  // namespace

const char* Imu::kindName() const {
    switch (kind_) {
        case IMU_MPU6050: return "MPU60x0/65xx";
        case IMU_LSM6:    return "LSM6";
        case IMU_QMI8658: return "QMI8658";
        case IMU_LIS3DH:  return "LIS3DH";
        case IMU_BMI160:  return "BMI160";
        case IMU_UNKNOWN: return "unknown IMU";
        default:          return "none";
    }
}

void Imu::scanBus() {
    Serial.printf("I2C scan on SDA GPIO%d / SCL GPIO%d:", TOUCH_SDA, TOUCH_SCL);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        if (!i2cPresent(addr)) continue;
        ++found;
        uint8_t who = 0;
        ImuKind kind = IMU_NONE;
        const char* name = identify(addr, who, kind);
        if (name && who) {
            Serial.printf("  0x%02X %s WHO=0x%02X", addr, name, who);
        } else if (name) {
            Serial.printf("  0x%02X %s", addr, name);
        } else if (who) {
            Serial.printf("  0x%02X id=0x%02X", addr, who);
        } else {
            Serial.printf("  0x%02X", addr);
        }
    }
    if (!found) Serial.print("  (none)");
    Serial.println();
}

void Imu::begin() {
    ok_ = false;
    kind_ = IMU_NONE;
    addr_ = 0;
    fails_ = 0;
    haveSmooth_ = false;
    smoothX_ = 0.0f;
    smoothY_ = 1.0f;

    scanBus();

    const uint8_t candidates[] = {0x68, 0x69, 0x6A, 0x6B, 0x18, 0x19, 0x28, 0x29};
    for (uint8_t a : candidates) {
        if (a == TOUCH_I2C_ADDR) continue;
        if (!i2cPresent(a)) continue;
        if (probeAddress(a, true) && configure()) {
            ok_ = true;
            Serial.printf("IMU %s online at 0x%02X (SDA GPIO%d, SCL GPIO%d)\n",
                          kindName(), addr_, TOUCH_SDA, TOUCH_SCL);
            printMapping();
            return;
        }
    }

    // Any other address that identified as an IMU.
    for (uint8_t addr = 1; addr < 127; ++addr) {
        if (addr == TOUCH_I2C_ADDR) continue;
        bool known = false;
        for (uint8_t a : candidates) if (a == addr) known = true;
        if (known) continue;
        if (!i2cPresent(addr)) continue;
        if (probeAddress(addr, true) && kind_ != IMU_NONE && kind_ != IMU_UNKNOWN &&
            configure()) {
            ok_ = true;
            Serial.printf("IMU %s online at 0x%02X (SDA GPIO%d, SCL GPIO%d)\n",
                          kindName(), addr_, TOUCH_SDA, TOUCH_SCL);
            printMapping();
            return;
        }
    }

    ok_ = false;
    kind_ = IMU_NONE;
    Serial.println("No IMU found on the touch bus. Fluid mode will use touch-drag as gravity.");
}

bool Imu::probeAddress(uint8_t addr, bool log) {
    uint8_t who = 0;
    ImuKind kind = IMU_NONE;
    const char* name = identify(addr, who, kind);
    if (kind == IMU_NONE) return false;
    if (kind == IMU_UNKNOWN) {
        if (log && name) {
            Serial.printf("I2C 0x%02X looked like %s (WHO=0x%02X), skipping\n",
                          addr, name, who);
        }
        return false;
    }
    addr_ = addr;
    kind_ = kind;
    if (log && name) {
        Serial.printf("I2C 0x%02X identified as %s (WHO=0x%02X)\n", addr, name, who);
    }
    return true;
}

bool Imu::configure() {
    switch (kind_) {
        case IMU_MPU6050:
            // Wake, gyro X clock. Chip boots in sleep.
            if (!writeReg(addr_, 0x6B, 0x01)) return false;
            delay(2);
            writeReg(addr_, 0x1C, 0x00);  // accel ±2 g
            writeReg(addr_, 0x1A, 0x03);  // DLPF ~44 Hz
            return true;
        case IMU_LSM6:
            // 104 Hz, ±2 g, default LPF. IF_INC is on by default.
            return writeReg(addr_, 0x10, 0x40);
        case IMU_QMI8658:
            writeReg(addr_, 0x60, 0xB0);  // soft reset (CTRL9 / reset variants)
            delay(10);
            writeReg(addr_, 0x02, 0x60);  // little-endian, auto-increment (CTRL1)
            writeReg(addr_, 0x03, 0x05);  // CTRL2: ±2 g, 200 Hz? aFS=000 aODR=0101
            // Some QMI8658 maps: CTRL2=0x0A, CTRL7=0x08. Use 0x0A / 0x08.
            writeReg(addr_, 0x0A, 0x05);  // accel ±2 g, ~200 Hz
            return writeReg(addr_, 0x08, 0x01);  // CTRL7 accel enable (some docs: 0x08)
        case IMU_LIS3DH:
            writeReg(addr_, 0x20, 0x57);  // 100 Hz, XYZ on
            writeReg(addr_, 0x23, 0x08);  // high-res, ±2 g
            return true;
        case IMU_BMI160:
            writeReg(addr_, 0x7E, 0xB6);  // soft reset
            delay(10);
            writeReg(addr_, 0x7E, 0x11);  // acc normal mode
            delay(50);
            writeReg(addr_, 0x40, 0x28);  // acc 100 Hz, ±2 g (ACC_CONF / ACC_RANGE split)
            writeReg(addr_, 0x41, 0x03);  // ±2 g
            return true;
        default:
            return false;
    }
}

bool Imu::readAccel(float& ax, float& ay, float& az) {
    uint8_t buf[6];
    const float lsb = 16384.0f;  // ±2 g for all the configs above
    switch (kind_) {
        case IMU_MPU6050:
            if (!readRegs(addr_, 0x3B, buf, 6)) return false;
            ax = be16(buf) / lsb;
            ay = be16(buf + 2) / lsb;
            az = be16(buf + 4) / lsb;
            return true;
        case IMU_LSM6:
            if (!readRegs(addr_, 0x28, buf, 6)) return false;
            ax = le16(buf) / lsb;
            ay = le16(buf + 2) / lsb;
            az = le16(buf + 4) / lsb;
            return true;
        case IMU_QMI8658:
            // AX_L at 0x35 on the usual QMI8658 map
            if (!readRegs(addr_, 0x35, buf, 6)) return false;
            ax = le16(buf) / lsb;
            ay = le16(buf + 2) / lsb;
            az = le16(buf + 4) / lsb;
            return true;
        case IMU_LIS3DH:
            if (!readRegs(addr_, 0x28 | 0x80, buf, 6)) return false;
            ax = le16(buf) / lsb;
            ay = le16(buf + 2) / lsb;
            az = le16(buf + 4) / lsb;
            return true;
        case IMU_BMI160:
            if (!readRegs(addr_, 0x12, buf, 6)) return false;  // ACC_X_LSB
            ax = le16(buf) / lsb;
            ay = le16(buf + 2) / lsb;
            az = le16(buf + 4) / lsb;
            return true;
        default:
            return false;
    }
}

void Imu::mapToScreen(float ax, float ay, float az, float& gx, float& gy) const {
    const int* m = kMaps[mapIndex_];
    gx = pick(ax, ay, az, m[0]) * (float)m[2];
    gy = pick(ax, ay, az, m[1]) * (float)m[3];
}

void Imu::cycleMapping() {
    mapIndex_ = (mapIndex_ + 1) % kMapCount;
    haveSmooth_ = false;
    printMapping();
}

void Imu::printMapping() const {
    const int* m = kMaps[mapIndex_];
    const char axis[] = {'X', 'Y', 'Z'};
    Serial.printf("IMU map %d: screenX=%c%c  screenY=%c%c\n",
                  mapIndex_,
                  m[2] < 0 ? '-' : '+', axis[m[0]],
                  m[3] < 0 ? '-' : '+', axis[m[1]]);
}

bool Imu::read(float& gx, float& gy) {
    if (!ok_) {
        gx = 0.0f;
        gy = 1.0f;
        return false;
    }
    float ax, ay, az;
    if (!readAccel(ax, ay, az)) {
        if (++fails_ >= 8) {
            ok_ = false;
            Serial.println("IMU stopped answering. Gravity-down until it returns.");
        }
        if (haveSmooth_) {
            gx = smoothX_;
            gy = smoothY_;
            return true;
        }
        gx = 0.0f;
        gy = 1.0f;
        return false;
    }
    fails_ = 0;

    float sx, sy;
    mapToScreen(ax, ay, az, sx, sy);
    sx = clampf(sx, -3.0f, 3.0f);
    sy = clampf(sy, -3.0f, 3.0f);

    if (!haveSmooth_) {
        smoothX_ = sx;
        smoothY_ = sy;
        haveSmooth_ = true;
    } else {
        const float k = 0.35f;
        smoothX_ += (sx - smoothX_) * k;
        smoothY_ += (sy - smoothY_) * k;
    }
    gx = smoothX_;
    gy = smoothY_;
    return true;
}

#endif  // BOARD_FNK0104
