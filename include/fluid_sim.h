#pragma once

#if defined(BOARD_FNK0104)

#include <stdint.h>

// FLIP / PIC water on a MAC grid, ported from esp32-fluid-matrix
// (Müller's Ten Minute Physics method). Positions are in cell units,
// +X right, +Y down. Gravity is passed in already mapped to screen axes.
//
// Interior tank is 64 x 48 cells (5 screen pixels each) so the fluid
// fills the 320x240 panel. That is a bit finer than yoRadio's 46x34
// screensaver grid, coarse enough (with a core-0 solver task) to
// target ~30 fps. Hot buffers prefer internal RAM.

class FluidSim {
 public:
    static const int kNX = 66;           // interior 64 + solid ring
    static const int kNY = 50;
    static const int kInteriorX = 64;
    static const int kInteriorY = 48;
    static const int kCells = kNX * kNY;
    static const int kMaxParticles = 1800;
    static const int kViewW = 160;       // published colour field (2x2 on screen)
    static const int kViewH = 120;

    bool begin(int particleCount = 1600, float flipRatio = 0.90f);
    void reseed();
    void stir();
    void step(float gx, float gy, float dt);
    // Colour by velocity (hue) and depth (brightness). Writes SCREEN pixels.
    void render(uint16_t* framebuffer, int screenW, int screenH);
    // Build the 64x48 colour field into dest (or the internal buffer).
    void prepareColors(uint16_t* dest = nullptr);
    // Bilinear-upsample the 64x48 field into a 160x120 snapshot for the
    // display task. The display then does a cheap 2x nearest expand.
    void prepareView(uint16_t* dest, int viewW = kViewW, int viewH = kViewH);
    // Blit one screen band from a 160x120 view (2x2 nearest) or, if colors
    // is a 64x48 field, bilinear-upsample. swapBytes pre-swaps for ILI9341.
    void renderBand(uint16_t* dest, int y0, int h, int screenW, bool swapBytes,
                    const uint16_t* colors = nullptr);

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
    void prepareSolverCells();
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

    // Cached P->G stencils, reused by G->P (Vateva). Two components.
    struct TransferStencil {
        int base;
        float w00, w10, w11, w01;
    };
    TransferStencil* stenU_ = nullptr;
    TransferStencil* stenV_ = nullptr;

    // Wall coefficients hoisted out of the pressure iteration loop (Vateva).
    struct SolverCell {
        int index;
        float divOff;
        float fL, fR, fD, fU;
    };
    SolverCell* solverCells_ = nullptr;
    int solverCount_ = 0;

    // 64x48 colour field, used when prepareColors() is called without a dest.
    uint16_t* cellColor_ = nullptr;
};

#endif  // BOARD_FNK0104
