#pragma once

#if defined(BOARD_FNK0104)

#include <stdint.h>

// FLIP / PIC water on a MAC grid, ported from esp32-fluid-matrix
// (Müller's Ten Minute Physics method). Positions are in cell units,
// +X right, +Y down. Gravity is passed in already mapped to screen axes.
//
// Interior tank is 80 x 60 cells (4 screen pixels each) so the fluid
// fills the 320x240 panel. Buffers live in PSRAM.

class FluidSim {
 public:
    static const int kNX = 82;           // interior 80 + solid ring
    static const int kNY = 62;
    static const int kInteriorX = 80;
    static const int kInteriorY = 60;
    static const int kCells = kNX * kNY;
    static const int kMaxParticles = 2800;

    bool begin(int particleCount = 2400, float flipRatio = 0.90f);
    void reseed();
    void stir();
    void step(float gx, float gy, float dt);
    // Colour by velocity (hue) and depth (brightness). Writes SCREEN pixels.
    void render(uint16_t* framebuffer, int screenW, int screenH);

    int count() const { return count_; }
    bool ready() const { return ready_; }
    float restDensity() const { return restDensity_; }

 private:
    void seed(int count);
    void integrate(float ax, float ay, float dt);
    void pushApart(int iters);
    void walls();
    void transferToGrid();
    void transferToParticles();
    void updateDensity();
    void solvePressure(float dt);
    void zeroSolidFaces();
    float faceHasFluid(int i, int j, int component) const;
    int idx(int i, int j) const { return i * kNY + j; }

    bool ready_ = false;
    int count_ = 0;
    int seedCount_ = 0;
    float flipRatio_ = 0.90f;
    float overRelax_ = 1.9f;
    float driftK_ = 16.0f;
    float restDensity_ = 0.0f;
    float h_ = 1.0f;
    float invH_ = 1.0f;
    float pInv_ = 1.0f;
    int pNumX_ = 1;
    int pNumY_ = 1;
    int pNumCells_ = 1;

    float* u_ = nullptr;
    float* v_ = nullptr;
    float* prevU_ = nullptr;
    float* prevV_ = nullptr;
    float* du_ = nullptr;
    float* dv_ = nullptr;
    float* density_ = nullptr;
    uint8_t* solid_ = nullptr;
    uint8_t* cellType_ = nullptr;

    float* px_ = nullptr;
    float* py_ = nullptr;
    float* pvx_ = nullptr;
    float* pvy_ = nullptr;

    int* numCell_ = nullptr;
    int* firstCell_ = nullptr;
    int* cellIds_ = nullptr;

    // 80x60 colour field, reused each frame for the upsample.
    uint16_t* cellColor_ = nullptr;
};

#endif  // BOARD_FNK0104
