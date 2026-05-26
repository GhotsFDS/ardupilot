/*
 * P8.5b.4 — AP_MantaShark control allocator (16 MSAK_ params wired)
 *
 * Defaults match b.3 hardcoded values, so SITL smoke test must reproduce
 * b.3 output bit-equivalent until user changes any MSAK_ param.
 *
 * 16 params per P8.5 design v3 §3 (MSAK_CTRL_EN NOT registered, P8.7+).
 */

#include "AP_MantaShark.h"

#include <AP_Math/AP_Math.h>
#include <algorithm>
#include <cmath>

AP_MantaShark *AP_MantaShark::_singleton = nullptr;

namespace {
constexpr float DEG2RAD_F = 0.017453292519943295f;

// Static tilt-key-index lookup (which tilts[] entry each column reads)
// Per P8.4 §1.5 canonical mapping.
constexpr int TILT_KEY[AP_MantaShark::N_AX] = {
    AP_MantaShark::TILT_SGRP,   // KS
    AP_MantaShark::TILT_DF,     // KDF
    AP_MantaShark::TILT_TL,     // KT_L
    AP_MantaShark::TILT_TR,     // KT_R
    AP_MantaShark::TILT_RD,     // KRD
};

// Yaw arm sign: only KT_L/KT_R contribute to Mz (yaw_gain = 1.0), others 0
constexpr float YAW_ARM[AP_MantaShark::N_AX] = {
    0.0f,   // KS
    0.0f,   // KDF
    1.0f,   // KT_L
    1.0f,   // KT_R
    0.0f,   // KRD
};
} // anon

// ───── AP_Param table (16 entries, per P8.5 design v3 §3) ─────
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

    // @Param: KDF_GAIN
    // @DisplayName: KDF group thrust gain
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("KDF_GAIN", 2, AP_MantaShark, _kdf_gain, 2.0f),

    // @Param: KT_GAIN
    // @DisplayName: KT_L/KT_R aggregate thrust gain (共用)
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("KT_GAIN", 3, AP_MantaShark, _kt_gain, 2.0f),

    // @Param: KRD_GAIN
    // @DisplayName: KRD group thrust gain
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("KRD_GAIN", 4, AP_MantaShark, _krd_gain, 2.0f),

    // @Param: KS_X
    // @DisplayName: KS x_m position (forward+, ArduPilot frame)
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("KS_X", 5, AP_MantaShark, _ks_x, 0.79f),

    // @Param: KDF_X
    // @DisplayName: KDF x_m position
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("KDF_X", 6, AP_MantaShark, _kdf_x, 0.80f),

    // @Param: KT_X
    // @DisplayName: KT_L/KT_R x_m position (共用)
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("KT_X", 7, AP_MantaShark, _kt_x, 0.05f),

    // @Param: KRD_X
    // @DisplayName: KRD x_m position
    // @Range: -2 2
    // @User: Advanced
    AP_GROUPINFO("KRD_X", 8, AP_MantaShark, _krd_x, -0.50f),

    // @Param: KT_Y
    // @DisplayName: KT |y_m| (对称, KT_L y=-this KT_R y=+this)
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("KT_Y", 9, AP_MantaShark, _kt_y, 0.38f),

    // @Param: W_FX
    // @DisplayName: WLS row weight Fx
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("W_FX", 10, AP_MantaShark, _w_fx, 1.0f),

    // @Param: W_FZ
    // @DisplayName: WLS row weight Fz (protect lift)
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("W_FZ", 11, AP_MantaShark, _w_fz, 8.0f),

    // @Param: W_MY
    // @DisplayName: WLS row weight My (protect pitch)
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("W_MY", 12, AP_MantaShark, _w_my, 12.0f),

    // @Param: W_MZ
    // @DisplayName: WLS row weight Mz (yaw)
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("W_MZ", 13, AP_MantaShark, _w_mz, 2.0f),

    // @Param: DAMP
    // @DisplayName: L2 damping (Tikhonov regularizer)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DAMP", 14, AP_MantaShark, _damping, 1.0e-4f),

    // @Param: MAX_ITER
    // @DisplayName: Active-set max iterations
    // @Range: 1 32
    // @User: Advanced
    AP_GROUPINFO("MAX_ITER", 15, AP_MantaShark, _max_iter, 8),

    // @Param: LOG_RATE
    // @DisplayName: BIN log Hz (lua hot-path uses for decimation)
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("LOG_RATE", 16, AP_MantaShark, _log_rate, 50),

    AP_GROUPEND
};

AP_MantaShark::AP_MantaShark() {
    if (_singleton != nullptr) {
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// ───── calibration accessors ─────
float AP_MantaShark::get_gain(int col) const {
    switch (col) {
        case COL_KS:   return _ks_gain.get();
        case COL_KDF:  return _kdf_gain.get();
        case COL_KT_L: return _kt_gain.get();
        case COL_KT_R: return _kt_gain.get();   // KT_L/KT_R 共用
        case COL_KRD:  return _krd_gain.get();
    }
    return 0.0f;
}
float AP_MantaShark::get_x(int col) const {
    switch (col) {
        case COL_KS:   return _ks_x.get();
        case COL_KDF:  return _kdf_x.get();
        case COL_KT_L: return _kt_x.get();
        case COL_KT_R: return _kt_x.get();
        case COL_KRD:  return _krd_x.get();
    }
    return 0.0f;
}
// Sanitized weight accessor (P8.5b.4a). Used by Tuner / debug paths.
// solve() uses identical clamp inline (see hot path) — keep in sync.
float AP_MantaShark::get_weight(int row) const {
    auto sanitize = [](float v, float fallback) -> float {
        if (!isfinite(v) || v < 0.0f || v > 100.0f) return fallback;
        return v;
    };
    switch (row) {
        case ROW_FX: return sanitize(_w_fx.get(), 1.0f);
        case ROW_FZ: return sanitize(_w_fz.get(), 8.0f);
        case ROW_MY: return sanitize(_w_my.get(), 12.0f);
        case ROW_MZ: return sanitize(_w_mz.get(), 2.0f);
    }
    return 1.0f;
}

// ───── build_matrix (analytical, body-frame, params runtime) ─────
// Per actuator i:
//   Fx_i = g_i × sin(θ_i)
//   Fz_i = -g_i × cos(θ_i)
//   My_i = -x_i × Fz_i
//   Mz_i = -y_i × Fx_i × yaw_gain_i
void AP_MantaShark::build_matrix(float A_out[N_ROWS * N_COLS], const State &s) const {
    for (int col = 0; col < N_COLS; ++col) {
        float gain = get_gain(col);
        float x_m  = get_x(col);
        // y_m: KT_L = -kt_y, KT_R = +kt_y, 其他 0
        float y_m  = 0.0f;
        if (col == COL_KT_L) y_m = -_kt_y.get();
        else if (col == COL_KT_R) y_m = +_kt_y.get();
        float yaw_g = YAW_ARM[col];

        float theta_rad = s.tilts[TILT_KEY[col]] * DEG2RAD_F;
        float st = sinf(theta_rad);
        float ct = cosf(theta_rad);
        float fx = gain * st;
        float fz = -gain * ct;
        float my = -x_m * fz;
        float mz = -y_m * fx * yaw_g;
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
            if (fabs(f) < 1.0e-300) continue;
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

// ───── solve (priority WLS + active-set, params now runtime-read) ─────
void AP_MantaShark::solve(const State &s_in, const Demand &d, Output &out) {
    // base_k sanitize
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

    // P8.5b.4a (gpt5 cdaeaf6 review): read-time sanitize for weights/damping/max_iter.
    // @Range metadata 只在 Tuner UI / preflight 提示, runtime param:set 可绕过.
    // NaN/Inf/负数会让 normal equations 奇异 → STATUS_SINGULAR → 全 0 输出 (污染 MSK10).
    // Clamp to safe ranges here; fall back to b.3-equivalent defaults if param out-of-bounds.
    auto sanitize_w = [](float v, float fallback) -> double {
        if (!isfinite(v) || v < 0.0f || v > 100.0f) return (double)fallback;
        return (double)v;
    };
    const double weights[N_ROWS] = {
        sanitize_w(_w_fx.get(), 1.0f),
        sanitize_w(_w_fz.get(), 8.0f),
        sanitize_w(_w_my.get(), 12.0f),
        sanitize_w(_w_mz.get(), 2.0f),
    };
    const double demand[N_ROWS] = {d.fx, d.fz, d.my, d.mz};

    float damp_raw = _damping.get();
    double damping;
    if (!isfinite(damp_raw) || damp_raw < 0.0f || damp_raw > 1.0f) {
        damping = 1.0e-4;   // b.3 default fallback
    } else {
        damping = (double)damp_raw;
    }

    int iter_raw = (int)_max_iter.get();
    int max_iter = (iter_raw >= 1 && iter_raw <= 32) ? iter_raw : 8;   // b.3 default 8

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

    for (int iter = 0; iter < max_iter; ++iter) {
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
        if (!solve_wls(A, weights, b, free_cols, num_free, damping, dk_free)) {
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
