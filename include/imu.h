#pragma once

#if defined(BOARD_FNK0104)

#include <stdint.h>

enum ImuKind : uint8_t {
    IMU_NONE = 0,
    IMU_MPU6050,
    IMU_LSM6,
    IMU_QMI8658,
    IMU_LIS3DH,
    IMU_BMI160,
    IMU_UNKNOWN
};

// IMU on the same I2C bus as the FT6336 (SDA 16 / SCL 15). Missing
// hardware is not fatal: read() then reports gravity straight down
// and ok() stays false. Touch-drag can stand in as gravity.
class Imu {
 public:
    void begin();
    // Full bus scan plus WHO_AM_I notes. Safe to call anytime.
    void scanBus();
    bool ok() const { return ok_; }
    ImuKind kind() const { return kind_; }
    uint8_t address() const { return addr_; }
    const char* kindName() const;
    // Screen-space acceleration in g. +X right, +Y down. Returns false
    // when the sample is the fallback (missing or a failed read).
    bool read(float& gx, float& gy);
    // Cycle through common axis mappings (serial 'm').
    void cycleMapping();
    void printMapping() const;
    // Toggle only the screen-X sign (serial 'x'), leaving screen Y alone.
    void flipX();
    // Last raw accelerometer sample in g. False if no successful raw read yet.
    bool lastRaw(float& ax, float& ay, float& az) const;
    bool readRaw(float& ax, float& ay, float& az);

 private:
    bool ok_ = false;
    ImuKind kind_ = IMU_NONE;
    uint8_t addr_ = 0;
    int fails_ = 0;
    int mapIndex_ = 0;
    int xSign_ = 1;  // extra screen-X flip, applied after the map
    float smoothX_ = 0.0f;
    float smoothY_ = 1.0f;
    bool haveSmooth_ = false;
    float rawX_ = 0.0f, rawY_ = 0.0f, rawZ_ = 0.0f;
    bool haveRaw_ = false;

    bool probeAddress(uint8_t addr, bool log);
    bool configure();
    bool readAccel(float& ax, float& ay, float& az);
    void mapToScreen(float ax, float ay, float az, float& gx, float& gy) const;
};

#endif  // BOARD_FNK0104
