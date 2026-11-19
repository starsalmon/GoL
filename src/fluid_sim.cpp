#include "board_config.h"

#if defined(BOARD_FNK0104)

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("O2")
#endif

#include "fluid_sim.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <esp_heap_caps.h>

namespace {

const int AIR = 0;
const int FLUID = 1;
const int SOLID = 2;

const float kParticleRadius = 0.46f;
const float kGravityAccel = 160.0f;   // cells / s^2 at 1 g (~640 px/s^2)
const float kMaxSpeed = 48.0f;        // 0.8 cell per 1/60 s frame
const int kPressureIters = 12;
const int kSeparateIters = 2;
const float kInvOpen[5] = {0.0f, 1.0f, 0.5f, 1.0f / 3.0f, 0.25f};
const float kTinyD2 = 1.0e-8f;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline int fastFloor(float v) {
    const int t = (int)v;
    return (v < (float)t) ? t - 1 : t;
}

inline void clampSpeed(float& vx, float& vy, float maxS) {
    const float s2 = vx * vx + vy * vy;
    const float maxS2 = maxS * maxS;
    if (s2 > maxS2) {
        const float s = sqrtf(s2);
        vx *= maxS / s;
        vy *= maxS / s;
    }
}

inline float halfGapOverDist(float d2, float minDist) {
    const float d = sqrtf(d2);
    return 0.5f * (minDist - d) / d;
}

void* allocPsram(size_t n) {
    void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = malloc(n);
    return p;
}

uint16_t hsvTo565(uint16_t h, uint8_t s, uint8_t v) {
    h = (uint16_t)(h % 360);
    uint8_t region = h / 60;
    uint16_t remainder = (uint16_t)((h - region * 60) * 255 / 60);
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
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

}  // namespace

bool FluidSim::begin(int particleCount, float flipRatio) {
    auto take = [](void** slot, size_t n) -> bool {
        *slot = allocPsram(n);
        return *slot != nullptr;
    };
    if (!u_) {
        bool ok = true;
        ok = ok && take((void**)&u_, sizeof(float) * kCells);
        ok = ok && take((void**)&v_, sizeof(float) * kCells);
        ok = ok && take((void**)&prevU_, sizeof(float) * kCells);
        ok = ok && take((void**)&prevV_, sizeof(float) * kCells);
        ok = ok && take((void**)&du_, sizeof(float) * kCells);
        ok = ok && take((void**)&dv_, sizeof(float) * kCells);
        ok = ok && take((void**)&density_, sizeof(float) * kCells);
        ok = ok && take((void**)&solid_, kCells);
        ok = ok && take((void**)&cellType_, kCells);
        ok = ok && take((void**)&px_, sizeof(float) * kMaxParticles);
        ok = ok && take((void**)&py_, sizeof(float) * kMaxParticles);
        ok = ok && take((void**)&pvx_, sizeof(float) * kMaxParticles);
        ok = ok && take((void**)&pvy_, sizeof(float) * kMaxParticles);
        const int hashX = kNX + 1;
        const int hashY = kNY + 1;
        const int hashCells = hashX * hashY;
        ok = ok && take((void**)&numCell_, sizeof(int) * hashCells);
        ok = ok && take((void**)&firstCell_, sizeof(int) * (hashCells + 1));
        ok = ok && take((void**)&cellIds_, sizeof(int) * kMaxParticles);
        ok = ok && take((void**)&cellColor_, sizeof(uint16_t) * kInteriorX * kInteriorY);
        if (!ok) {
            Serial.println("Fluid sim alloc failed");
            ready_ = false;
            return false;
        }
        memset(u_, 0, sizeof(float) * kCells);
        memset(v_, 0, sizeof(float) * kCells);
        memset(prevU_, 0, sizeof(float) * kCells);
        memset(prevV_, 0, sizeof(float) * kCells);
        memset(du_, 0, sizeof(float) * kCells);
        memset(dv_, 0, sizeof(float) * kCells);
        memset(density_, 0, sizeof(float) * kCells);
        memset(px_, 0, sizeof(float) * kMaxParticles);
        memset(py_, 0, sizeof(float) * kMaxParticles);
        memset(pvx_, 0, sizeof(float) * kMaxParticles);
        memset(pvy_, 0, sizeof(float) * kMaxParticles);

        for (int i = 0; i < kNX; ++i) {
            for (int j = 0; j < kNY; ++j) {
                const bool border = (i == 0 || j == 0 || i == kNX - 1 || j == kNY - 1);
                solid_[idx(i, j)] = border ? 0 : 1;
            }
        }
    }

    h_ = 1.0f;
    invH_ = 1.0f;
    flipRatio_ = clampf(flipRatio, 0.40f, 0.98f);
    pInv_ = 1.0f / (2.2f * kParticleRadius);
    const float width = (float)(kNX - 1) * h_;
    const float height = (float)(kNY - 1) * h_;
    pNumX_ = fastFloor(width * pInv_) + 1;
    pNumY_ = fastFloor(height * pInv_) + 1;
    const int hashX = kNX + 1;
    const int hashY = kNY + 1;
    if (pNumX_ > hashX) pNumX_ = hashX;
    if (pNumY_ > hashY) pNumY_ = hashY;
    if (pNumX_ < 1) pNumX_ = 1;
    if (pNumY_ < 1) pNumY_ = 1;
    pNumCells_ = pNumX_ * pNumY_;

    if (particleCount < 200) particleCount = 200;
    if (particleCount > kMaxParticles) particleCount = kMaxParticles;
    seedCount_ = particleCount;
    seed(particleCount);
    ready_ = true;
    Serial.printf("Fluid sim ready: %d particles, %dx%d MAC grid\n",
                  count_, kInteriorX, kInteriorY);
    return true;
}

void FluidSim::reseed() { seed(seedCount_ > 0 ? seedCount_ : 2400); }

void FluidSim::stir() {
    for (int i = 0; i < count_; ++i) {
        pvx_[i] += ((float)random(-80, 81)) * 0.15f;
        pvy_[i] += ((float)random(-80, 81)) * 0.15f;
    }
}

void FluidSim::seed(int count) {
    if (count > kMaxParticles) count = kMaxParticles;
    restDensity_ = 0.0f;
    memset(density_, 0, sizeof(float) * kCells);
    memset(u_, 0, sizeof(float) * kCells);
    memset(v_, 0, sizeof(float) * kCells);

    const float r = kParticleRadius;
    const float dx = 2.0f * r;
    const float dy = 0.8660254f * dx;
    const float minX = h_ + r;
    const float maxX = (float)(kNX - 1) * h_ - r;
    const float minY = h_ + r;
    const float maxY = (float)(kNY - 1) * h_ - r;
    const float usableW = maxX - minX;

    int cols = fastFloor(usableW / dx);
    if (cols < 1) cols = 1;
    const int rows = (count + cols - 1) / cols;
    const float blockW = (float)cols * dx;
    const float blockH = (float)rows * dy;
    float x0 = minX + 0.5f * (usableW - blockW);
    if (x0 < minX) x0 = minX;
    float y0 = maxY - blockH;  // pool at the bottom (large Y)
    if (y0 < minY) y0 = minY;

    int n = 0;
    for (int row = 0; n < count; ++row) {
        const float stagger = (row & 1) ? 0.5f * dx : 0.0f;
        for (int col = 0; col < cols && n < count; ++col) {
            const float x = x0 + r + (float)col * dx + stagger;
            const float y = y0 + r + (float)row * dy;
            px_[n] = clampf(x, minX, maxX);
            py_[n] = clampf(y, minY, maxY);
            pvx_[n] = 0.0f;
            pvy_[n] = 0.0f;
            ++n;
        }
    }
    count_ = n;
}

void FluidSim::step(float gx, float gy, float dt) {
    if (!ready_ || count_ <= 0 || !(dt > 0.0f)) return;
    gx = clampf(gx, -3.0f, 3.0f);
    gy = clampf(gy, -3.0f, 3.0f);
    const float ax = gx * kGravityAccel;
    const float ay = gy * kGravityAccel;
    integrate(ax, ay, dt);
    pushApart(kSeparateIters);
    walls();
    transferToGrid();
    updateDensity();
    solvePressure(dt);
    transferToParticles();
}

void FluidSim::integrate(float ax, float ay, float dt) {
    for (int i = 0; i < count_; ++i) {
        pvx_[i] += ax * dt;
        pvy_[i] += ay * dt;
        clampSpeed(pvx_[i], pvy_[i], kMaxSpeed);
        px_[i] += pvx_[i] * dt;
        py_[i] += pvy_[i] * dt;
    }
    walls();
}

void FluidSim::walls() {
    const float r = kParticleRadius;
    const float minX = h_ + r;
    const float maxX = (float)(kNX - 1) * h_ - r;
    const float minY = h_ + r;
    const float maxY = (float)(kNY - 1) * h_ - r;
    for (int i = 0; i < count_; ++i) {
        if (px_[i] < minX) {
            px_[i] = minX;
            if (pvx_[i] < 0.0f) pvx_[i] = 0.0f;
        } else if (px_[i] > maxX) {
            px_[i] = maxX;
            if (pvx_[i] > 0.0f) pvx_[i] = 0.0f;
        }
        if (py_[i] < minY) {
            py_[i] = minY;
            if (pvy_[i] < 0.0f) pvy_[i] = 0.0f;
        } else if (py_[i] > maxY) {
            py_[i] = maxY;
            if (pvy_[i] > 0.0f) pvy_[i] = 0.0f;
        }
    }
}

void FluidSim::pushApart(int iters) {
    if (iters <= 0 || count_ <= 1 || pNumCells_ <= 0) return;
    memset(numCell_, 0, sizeof(int) * (size_t)pNumCells_);
    for (int i = 0; i < count_; ++i) {
        const int xi = clampi(fastFloor(px_[i] * pInv_), 0, pNumX_ - 1);
        const int yi = clampi(fastFloor(py_[i] * pInv_), 0, pNumY_ - 1);
        numCell_[xi * pNumY_ + yi]++;
    }
    int running = 0;
    for (int i = 0; i < pNumCells_; ++i) {
        running += numCell_[i];
        firstCell_[i] = running;
    }
    firstCell_[pNumCells_] = running;
    for (int i = 0; i < count_; ++i) {
        const int xi = clampi(fastFloor(px_[i] * pInv_), 0, pNumX_ - 1);
        const int yi = clampi(fastFloor(py_[i] * pInv_), 0, pNumY_ - 1);
        const int cell = xi * pNumY_ + yi;
        firstCell_[cell]--;
        cellIds_[firstCell_[cell]] = i;
    }

    const float minDist = 2.0f * kParticleRadius;
    const float minDist2 = minDist * minDist;
    for (int iter = 0; iter < iters; ++iter) {
        for (int i = 0; i < count_; ++i) {
            const int pxi = fastFloor(px_[i] * pInv_);
            const int pyi = fastFloor(py_[i] * pInv_);
            const int x0 = pxi - 1 > 0 ? pxi - 1 : 0;
            const int y0 = pyi - 1 > 0 ? pyi - 1 : 0;
            const int x1 = pxi + 1 < pNumX_ - 1 ? pxi + 1 : pNumX_ - 1;
            const int y1 = pyi + 1 < pNumY_ - 1 ? pyi + 1 : pNumY_ - 1;
            if (x1 < x0 || y1 < y0) continue;
            for (int xi = x0; xi <= x1; ++xi) {
                for (int yi = y0; yi <= y1; ++yi) {
                    const int cell = xi * pNumY_ + yi;
                    const int first = firstCell_[cell];
                    const int last = firstCell_[cell + 1];
                    for (int k = first; k < last; ++k) {
                        const int id = cellIds_[k];
                        if (id == i) continue;
                        float dx = px_[id] - px_[i];
                        float dy = py_[id] - py_[i];
                        float d2 = dx * dx + dy * dy;
                        if (d2 > minDist2) continue;
                        if (d2 <= kTinyD2) {
                            dx = 0.01f;
                            dy = 0.0f;
                            d2 = dx * dx;
                        }
                        const float s = halfGapOverDist(d2, minDist);
                        dx *= s;
                        dy *= s;
                        px_[i] -= dx;
                        py_[i] -= dy;
                        px_[id] += dx;
                        py_[id] += dy;
                    }
                }
            }
        }
    }
}

void FluidSim::zeroSolidFaces() {
    for (int i = 0; i < kNX; ++i) {
        for (int j = 0; j < kNY; ++j) {
            const int c = idx(i, j);
            const bool solid = cellType_[c] == SOLID;
            if (solid || (i > 0 && cellType_[idx(i - 1, j)] == SOLID)) u_[c] = 0.0f;
            if (solid || (j > 0 && cellType_[idx(i, j - 1)] == SOLID)) v_[c] = 0.0f;
        }
    }
}

float FluidSim::faceHasFluid(int i, int j, int component) const {
    const int ni = component == 0 ? i - 1 : i;
    const int nj = component == 0 ? j : j - 1;
    const bool here =
        (i >= 0 && j >= 0 && i < kNX && j < kNY) && cellType_[idx(i, j)] != 0;
    const bool other = (ni >= 0 && nj >= 0 && ni < kNX && nj < kNY)
                           ? cellType_[idx(ni, nj)] != 0
                           : true;
    return (here || other) ? 1.0f : 0.0f;
}

void FluidSim::transferToGrid() {
    memset(u_, 0, sizeof(float) * kCells);
    memset(v_, 0, sizeof(float) * kCells);
    memset(du_, 0, sizeof(float) * kCells);
    memset(dv_, 0, sizeof(float) * kCells);
    for (int c = 0; c < kCells; ++c) {
        cellType_[c] = solid_[c] == 0 ? SOLID : AIR;
    }
    for (int p = 0; p < count_; ++p) {
        const int xi = clampi(fastFloor(px_[p] * invH_), 0, kNX - 1);
        const int yi = clampi(fastFloor(py_[p] * invH_), 0, kNY - 1);
        const int c = idx(xi, yi);
        if (cellType_[c] == AIR) cellType_[c] = FLUID;
    }

    const float h2 = 0.5f * h_;
    for (int component = 0; component < 2; ++component) {
        const float dx = component == 0 ? 0.0f : h2;
        const float dy = component == 0 ? h2 : 0.0f;
        float* f = component == 0 ? u_ : v_;
        float* w = component == 0 ? du_ : dv_;
        const float* vel = component == 0 ? pvx_ : pvy_;
        for (int p = 0; p < count_; ++p) {
            const float x = clampf(px_[p], h_, (float)(kNX - 1) * h_);
            const float y = clampf(py_[p], h_, (float)(kNY - 1) * h_);
            int x0 = fastFloor((x - dx) * invH_);
            int y0 = fastFloor((y - dy) * invH_);
            x0 = clampi(x0, 0, kNX - 2);
            y0 = clampi(y0, 0, kNY - 2);
            const float tx = ((x - dx) - (float)x0 * h_) * invH_;
            const float ty = ((y - dy) - (float)y0 * h_) * invH_;
            const int x1 = x0 + 1;
            const int y1 = y0 + 1;
            const float sx = 1.0f - tx;
            const float sy = 1.0f - ty;
            const float w00 = sx * sy, w10 = tx * sy, w11 = tx * ty, w01 = sx * ty;
            const float pv = vel[p];
            const int n00 = idx(x0, y0), n10 = idx(x1, y0);
            const int n11 = idx(x1, y1), n01 = idx(x0, y1);
            f[n00] += pv * w00; w[n00] += w00;
            f[n10] += pv * w10; w[n10] += w10;
            f[n11] += pv * w11; w[n11] += w11;
            f[n01] += pv * w01; w[n01] += w01;
        }
        for (int c = 0; c < kCells; ++c) {
            if (w[c] > 0.0f) f[c] /= w[c];
        }
    }
    zeroSolidFaces();
}

void FluidSim::updateDensity() {
    memset(density_, 0, sizeof(float) * kCells);
    const float h2 = 0.5f * h_;
    for (int p = 0; p < count_; ++p) {
        const float x = clampf(px_[p], h_, (float)(kNX - 1) * h_);
        const float y = clampf(py_[p], h_, (float)(kNY - 1) * h_);
        int x0 = clampi(fastFloor((x - h2) * invH_), 0, kNX - 2);
        int y0 = clampi(fastFloor((y - h2) * invH_), 0, kNY - 2);
        const float tx = ((x - h2) - (float)x0 * h_) * invH_;
        const float ty = ((y - h2) - (float)y0 * h_) * invH_;
        const int x1 = x0 + 1, y1 = y0 + 1;
        const float sx = 1.0f - tx, sy = 1.0f - ty;
        density_[idx(x0, y0)] += sx * sy;
        density_[idx(x1, y0)] += tx * sy;
        density_[idx(x1, y1)] += tx * ty;
        density_[idx(x0, y1)] += sx * ty;
    }
    if (restDensity_ == 0.0f) {
        float sum = 0.0f;
        int n = 0;
        for (int c = 0; c < kCells; ++c) {
            if (cellType_[c] == FLUID) {
                sum += density_[c];
                ++n;
            }
        }
        if (n > 0) restDensity_ = sum / (float)n;
    }
}

void FluidSim::solvePressure(float dt) {
    memcpy(prevU_, u_, sizeof(float) * kCells);
    memcpy(prevV_, v_, sizeof(float) * kCells);
    if (!(dt > 0.0f)) return;
    for (int iter = 0; iter < kPressureIters; ++iter) {
        for (int i = 1; i < kNX - 1; ++i) {
            for (int j = 1; j < kNY - 1; ++j) {
                const int center = idx(i, j);
                if (cellType_[center] != FLUID) continue;
                const int left = idx(i - 1, j);
                const int right = idx(i + 1, j);
                const int bottom = idx(i, j - 1);
                const int top = idx(i, j + 1);
                const float sx0 = (float)solid_[left];
                const float sx1 = (float)solid_[right];
                const float sy0 = (float)solid_[bottom];
                const float sy1 = (float)solid_[top];
                const float s = sx0 + sx1 + sy0 + sy1;
                if (s == 0.0f) continue;
                float div = u_[right] - u_[center] + v_[top] - v_[center];
                if (restDensity_ > 0.0f) {
                    const float compression = density_[center] - restDensity_;
                    if (compression > 0.0f) div -= driftK_ * compression;
                }
                float p = -div * kInvOpen[(int)s];
                p *= overRelax_;
                u_[center] -= sx0 * p;
                u_[right] += sx1 * p;
                v_[center] -= sy0 * p;
                v_[top] += sy1 * p;
            }
        }
    }
    zeroSolidFaces();
}

void FluidSim::transferToParticles() {
    const float h2 = 0.5f * h_;
    const float flip = flipRatio_;
    const float pic = 1.0f - flip;
    for (int component = 0; component < 2; ++component) {
        const float dx = component == 0 ? 0.0f : h2;
        const float dy = component == 0 ? h2 : 0.0f;
        const float* f = component == 0 ? u_ : v_;
        const float* prev = component == 0 ? prevU_ : prevV_;
        float* vel = component == 0 ? pvx_ : pvy_;
        for (int p = 0; p < count_; ++p) {
            const float x = clampf(px_[p], h_, (float)(kNX - 1) * h_);
            const float y = clampf(py_[p], h_, (float)(kNY - 1) * h_);
            int x0 = clampi(fastFloor((x - dx) * invH_), 0, kNX - 2);
            int y0 = clampi(fastFloor((y - dy) * invH_), 0, kNY - 2);
            const float tx = ((x - dx) - (float)x0 * h_) * invH_;
            const float ty = ((y - dy) - (float)y0 * h_) * invH_;
            const int x1 = x0 + 1, y1 = y0 + 1;
            const float sx = 1.0f - tx, sy = 1.0f - ty;
            const float w00 = sx * sy, w10 = tx * sy, w11 = tx * ty, w01 = sx * ty;
            const int n00 = idx(x0, y0), n10 = idx(x1, y0);
            const int n11 = idx(x1, y1), n01 = idx(x0, y1);
            const bool valid0 = faceHasFluid(x0, y0, component) > 0.0f;
            const bool valid1 = faceHasFluid(x1, y0, component) > 0.0f;
            const bool valid2 = faceHasFluid(x1, y1, component) > 0.0f;
            const bool valid3 = faceHasFluid(x0, y1, component) > 0.0f;
            const float a0 = valid0 ? w00 : 0.0f;
            const float a1 = valid1 ? w10 : 0.0f;
            const float a2 = valid2 ? w11 : 0.0f;
            const float a3 = valid3 ? w01 : 0.0f;
            const float wsum = a0 + a1 + a2 + a3;
            if (!(wsum > 0.0f)) continue;
            float picV = a0 * f[n00] + a1 * f[n10] + a2 * f[n11] + a3 * f[n01];
            float corr = a0 * (f[n00] - prev[n00]) + a1 * (f[n10] - prev[n10]) +
                         a2 * (f[n11] - prev[n11]) + a3 * (f[n01] - prev[n01]);
            if (!(valid0 && valid1 && valid2 && valid3)) {
                const float inv = 1.0f / wsum;
                picV *= inv;
                corr *= inv;
            }
            const float flipV = vel[p] + corr;
            vel[p] = pic * picV + flip * flipV;
        }
    }
    for (int i = 0; i < count_; ++i) clampSpeed(pvx_[i], pvy_[i], kMaxSpeed);
}

void FluidSim::render(uint16_t* framebuffer, int screenW, int screenH) {
    if (!ready_ || !framebuffer) return;
    const float rest = restDensity_ > 0.0f ? restDensity_ : 1.0f;
    const float invRest = 1.0f / rest;
    const float scaleX = (float)screenW / (float)kInteriorX;
    const float scaleY = (float)screenH / (float)kInteriorY;

    for (int i = 0; i < kInteriorX; ++i) {
        for (int j = 0; j < kInteriorY; ++j) {
            const int c = idx(i + 1, j + 1);
            const float d = density_[c] * invRest;
            const float speed = sqrtf(u_[c] * u_[c] + v_[c] * v_[c]);
            uint16_t color = 0;
            if (d > 0.08f) {
                // Blue water, shifting cyan/yellow with speed, brighter when deeper.
                int hue = 210 - (int)clampf(speed * 8.0f, 0.0f, 80.0f);
                if (hue < 0) hue += 360;
                uint8_t sat = (uint8_t)(255.0f - clampf(speed * 12.0f, 0.0f, 140.0f));
                uint8_t val = (uint8_t)clampf(40.0f + d * 200.0f, 20.0f, 255.0f);
                color = hsvTo565((uint16_t)hue, sat, val);
            }
            cellColor_[j * kInteriorX + i] = color;
        }
    }

    // Nearest-neighbour upsample (4x) — cheap and sharp on a 2.8" panel.
    const int pixX = (int)(scaleX + 0.5f);
    const int pixY = (int)(scaleY + 0.5f);
    for (int j = 0; j < kInteriorY; ++j) {
        for (int py = 0; py < pixY; ++py) {
            const int y = j * pixY + py;
            if (y >= screenH) continue;
            uint16_t* row = framebuffer + y * screenW;
            for (int i = 0; i < kInteriorX; ++i) {
                const uint16_t color = cellColor_[j * kInteriorX + i];
                const int x0 = i * pixX;
                for (int px = 0; px < pixX; ++px) {
                    const int x = x0 + px;
                    if (x < screenW) row[x] = color;
                }
            }
        }
    }

    // Particle highlights so spray and the free surface stay visible.
    for (int p = 0; p < count_; ++p) {
        const int x = (int)((px_[p] - 1.0f) * scaleX);
        const int y = (int)((py_[p] - 1.0f) * scaleY);
        if ((unsigned)x >= (unsigned)screenW || (unsigned)y >= (unsigned)screenH) continue;
        const float speed = sqrtf(pvx_[p] * pvx_[p] + pvy_[p] * pvy_[p]);
        uint8_t add = (uint8_t)clampf(80.0f + speed * 6.0f, 80.0f, 255.0f);
        uint16_t* pix = &framebuffer[y * screenW + x];
        // Lighten the pixel toward white without a full RGB convert.
        uint16_t c = *pix;
        int r = (c >> 11) & 0x1F;
        int g = (c >> 5) & 0x3F;
        int b = c & 0x1F;
        r = r + ((31 - r) * add) / 400;
        g = g + ((63 - g) * add) / 400;
        b = b + ((31 - b) * add) / 400;
        *pix = (uint16_t)((r << 11) | (g << 5) | b);
    }
}

#endif  // BOARD_FNK0104
