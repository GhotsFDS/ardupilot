/*
 * P8.5b.3 — AP_MantaShark control allocator (real algorithm)
 *
 * Ported verbatim from MantaShark/tools/allocator_cpp/AP_MantaShark_Allocator.cpp.
 * Parity verified: 1000 random cases vs Python prototype, max ΔK diff 1.6e-6.
 *
 * Solver: priority WLS + active-set clipping
 *   minimize  || W (A * dK - b) ||^2 + damping * || dK ||^2
 *   subject to  -base_k <= dK <= 1 - base_k
 */

#include "AP_MantaShark.h"

#include <AP_Math/AP_Math.h>
#include <algorithm>
#include <cmath>

AP_MantaShark *AP_MantaShark::_singleton = nullptr;

// ───── actuator model constants (calibration scalars, P8.4 v1 placeholders) ─────
// Position from MantaShark scripts/modules/actuators.lua (CAD → ArduPilot frame).
// thrust_gain 当前是 placeholder, P8.4 v2 + bench calibration 后填真值.
namespace {
struct ActuatorModel {
    int tilt_key_idx;    // index into State.tilts[]
    float x_m;           // forward + (ArduPilot frame)
    float y_m;           // right + (ArduPilot frame)
    float thrust_gain;   // calibration scalar
    float yaw_gain;      // 0 / 1
};

// Mirror tools/allocator_cpp default_models, indexed by Col enum
constexpr ActuatorModel ACTUATOR_MODELS[AP_MantaShark::N_AX] = {
    // tilt_key,                          x_m,    y_m,     thrust_gain, yaw_gain
    {AP_MantaShark::TILT_SGRP,            0.79f,  0.0f,    4.0f,        0.0f},  // KS
    {AP_MantaShark::TILT_DF,              0.80f,  0.0f,    2.0f,        0.0f},  // KDF
    {AP_MantaShark::TILT_TL,              0.05f, -0.38f,   2.0f,        1.0f},  // KT_L
    {AP_MantaShark::TILT_TR,              0.05f, +0.38f,   2.0f,        1.0f},  // KT_R
    {AP_MantaShark::TILT_RD,             -0.50f,  0.0f,    2.0f,        0.0f},  // KRD
};

// WLS row weights (priority — protect Fz/My over Fx, per gpt5 P8.0 design)
constexpr float W_FX = 1.0f, W_FZ = 8.0f, W_MY = 12.0f, W_MZ = 2.0f;
constexpr float DAMPING = 1.0e-4f;
constexpr int MAX_ITER = 8;
constexpr float DEG2RAD_F = 0.017453292519943295f;
} // anon

// ───── AP_Param table (P8.5b.3: 2 params placeholder, P8.5b.4 加完整 16) ─────
const AP_Param::GroupInfo AP_MantaShark::var_info[] = {
    // @Param: KS_GAIN
    // @DisplayName: KS group thrust gain (calibration scalar)
    // @Description: Effective thrust-per-K for KS (4 EDF). Bench-calibrated.
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("KS_GAIN", 0, AP_MantaShark, _ks_gain, 4.0f),

    // @Param: LOG_EN
    // @DisplayName: Allocator log enable
    // @Description: Enable MSK10/MSK11 BIN log of allocator output/input.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LOG_EN", 1, AP_MantaShark, _log_en, 1),

    AP_GROUPEND
};

AP_MantaShark::AP_MantaShark() {
    if (_singleton != nullptr) {
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// ───── build_matrix (analytical, body-frame) ─────
// Per actuator i:
//   Fx_i = g_i × sin(θ_i)
//   Fz_i = -g_i × cos(θ_i)   (Fz<0 = lift, ArduPilot convention)
//   My_i = -x_i × Fz_i
//   Mz_i = -y_i × Fx_i × yaw_gain_i
// theta_eff = tilts[i] (body frame, NOT + pitch_rad — gpt5 P8.4 review decision).
void AP_MantaShark::build_matrix(float A_out[N_ROWS * N_COLS], const State &s) const {
    for (int col = 0; col < N_COLS; ++col) {
        const auto &m = ACTUATOR_MODELS[col];
        float theta_rad = s.tilts[m.tilt_key_idx] * DEG2RAD_F;
        float st = sinf(theta_rad);
        float ct = cosf(theta_rad);
        float fx = m.thrust_gain * st;
        float fz = -m.thrust_gain * ct;
        float my = -m.x_m * fz;
        float mz = -m.y_m * fx * m.yaw_gain;
        A_out[ROW_FX * N_COLS + col] = fx;
        A_out[ROW_FZ * N_COLS + col] = fz;
        A_out[ROW_MY * N_COLS + col] = my;
        A_out[ROW_MZ * N_COLS + col] = mz;
    }
}

// ───── dense gauss solve (double precision, ≤ 5×5) ─────
static bool gauss_solve(double *M, double *b, int n) {
    for (int i = 0; i < n; ++i) {
        int piv = i;
        double piv_val = fabs(M[i * n + i]);
        for (int r = i + 1; r < n; ++r) {
            double v = fabs(M[r * n + i]);
            if (v > piv_val) { piv = r; piv_val = v; }
        }
        if (piv_val < 1.0e-12) return false;
        if (piv != i) {
            for (int c = 0; c < n; ++c) std::swap(M[i * n + c], M[piv * n + c]);
            std::swap(b[i], b[piv]);
        }
        double inv_diag = 1.0 / M[i * n + i];
        for (int c = i; c < n; ++c) M[i * n + c] *= inv_diag;
        b[i] *= inv_diag;
        for (int r = 0; r < n; ++r) {
            if (r == i) continue;
            double f = M[r * n + i];
            if (fabs(f) < 1.0e-300) continue;   // skip near-zero pivots (was f == 0.0, -Werror=float-equal)
            for (int c = i; c < n; ++c) M[r * n + c] -= f * M[i * n + c];
            b[r] -= f * b[i];
        }
    }
    return true;
}

// ───── WLS sub-solver (free variables only, double internally) ─────
static bool solve_wls(
    const double A_full[AP_MantaShark::N_ROWS * AP_MantaShark::N_COLS],
    const double weights[AP_MantaShark::N_ROWS],
    const double b[AP_MantaShark::N_ROWS],
    const int free_cols[AP_MantaShark::N_AX], int num_free,
    double damping,
    double dk_out[AP_MantaShark::N_AX])
{
    if (num_free == 0) return true;
    double AtWA[AP_MantaShark::N_AX * AP_MantaShark::N_AX] = {0};
    double AtWb[AP_MantaShark::N_AX] = {0};
    for (int i = 0; i < num_free; ++i) {
        int ci = free_cols[i];
        for (int j = 0; j < num_free; ++j) {
            int cj = free_cols[j];
            double sum = 0;
            for (int r = 0; r < AP_MantaShark::N_ROWS; ++r) {
                sum += weights[r] * weights[r]
                     * A_full[r * AP_MantaShark::N_COLS + ci]
                     * A_full[r * AP_MantaShark::N_COLS + cj];
            }
            if (i == j) sum += damping;
            AtWA[i * num_free + j] = sum;
        }
        double sumb = 0;
        for (int r = 0; r < AP_MantaShark::N_ROWS; ++r) {
            sumb += weights[r] * weights[r] * A_full[r * AP_MantaShark::N_COLS + ci] * b[r];
        }
        AtWb[i] = sumb;
    }
    if (!gauss_solve(AtWA, AtWb, num_free)) return false;
    for (int i = 0; i < num_free; ++i) dk_out[i] = AtWb[i];
    return true;
}

// ───── solve (priority WLS + active-set clipping) ─────
void AP_MantaShark::solve(const State &s_in, const Demand &d, Output &out) {
    // base_k sanitize (gpt5 P8.2 v2 review #4): clamp at solver entry
    State s = s_in;
    uint8_t status = STATUS_OK;
    for (int c = 0; c < N_AX; ++c) {
        if (s.base_k[c] < 0.0f) { s.base_k[c] = 0.0f; status |= STATUS_BASE_K_CLAMP; }
        else if (s.base_k[c] > 1.0f) { s.base_k[c] = 1.0f; status |= STATUS_BASE_K_CLAMP; }
    }

    float A_f[N_ROWS * N_COLS];
    build_matrix(A_f, s);
    double A[N_ROWS * N_COLS];
    for (int i = 0; i < N_ROWS * N_COLS; ++i) A[i] = A_f[i];

    const double weights[N_ROWS] = {W_FX, W_FZ, W_MY, W_MZ};
    const double demand[N_ROWS] = {d.fx, d.fz, d.my, d.mz};

    double dk[N_AX] = {0, 0, 0, 0, 0};
    double lo[N_AX], hi[N_AX];
    for (int c = 0; c < N_AX; ++c) {
        lo[c] = -s.base_k[c];
        hi[c] = 1.0 - s.base_k[c];
    }

    int free_cols[N_AX];
    bool fixed[N_AX] = {false, false, false, false, false};
    out.sat_lo = 0;
    out.sat_hi = 0;
    bool converged = false;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        int num_free = 0;
        for (int c = 0; c < N_AX; ++c) {
            if (!fixed[c]) free_cols[num_free++] = c;
        }

        double b[N_ROWS];
        for (int r = 0; r < N_ROWS; ++r) {
            b[r] = demand[r];
            for (int c = 0; c < N_AX; ++c) {
                if (fixed[c]) b[r] -= A[r * N_COLS + c] * dk[c];
            }
        }

        double dk_free[N_AX];
        if (!solve_wls(A, weights, b, free_cols, num_free, DAMPING, dk_free)) {
            status |= STATUS_SINGULAR;
            break;
        }

        for (int i = 0; i < num_free; ++i) {
            dk[free_cols[i]] = dk_free[i];
        }

        bool new_sat = false;
        for (int i = 0; i < num_free; ++i) {
            int c = free_cols[i];
            if (dk[c] < lo[c]) {
                dk[c] = lo[c];
                fixed[c] = true;
                out.sat_lo |= (1 << c);
                new_sat = true;
            } else if (dk[c] > hi[c]) {
                dk[c] = hi[c];
                fixed[c] = true;
                out.sat_hi |= (1 << c);
                new_sat = true;
            }
        }
        if (!new_sat) { converged = true; break; }
    }
    if (!converged && !(status & STATUS_SINGULAR)) {
        status |= STATUS_MAX_ITER;
    }

    for (int c = 0; c < N_AX; ++c) out.delta_k[c] = (float)dk[c];
    for (int r = 0; r < N_ROWS; ++r) {
        double acc = demand[r];
        for (int c = 0; c < N_AX; ++c) acc -= A[r * N_COLS + c] * dk[c];
        out.residual[r] = (float)acc;
    }
    out.status = status;
}

namespace AP {
    AP_MantaShark &mantashark() {
        return *AP_MantaShark::get_singleton();
    }
}
