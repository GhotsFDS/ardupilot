/*
 * P8.5b.4 — AP_MantaShark control allocator (16 MSAK_ params wired)
 *
 * Real allocator algorithm (priority WLS + active-set clipping).
 * Calibration via 16 MSAK_ params (defaults match b.3 hardcoded values).
 *
 * Phase: log-only (no control takeover). MSAK_CTRL_EN 不注册 — P8.7+ 才考虑.
 *
 * Matrix layout (HARD SPEC, see docs/P8_bridge/matrix_derivation.md §3.4-bis):
 *   A is 4 rows × 5 cols, ROW-MAJOR.  A[row * 5 + col]
 *     rows = [Fx, Fz, My, Mz]
 *     cols = [KS, KDF, KT_L, KT_R, KRD]
 */

#pragma once

#include <AP_Param/AP_Param.h>

class AP_MantaShark {
public:
    AP_MantaShark();

    CLASS_NO_COPY(AP_MantaShark);

    static AP_MantaShark *get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // ───── public allocator API ─────
    static constexpr int N_ROWS = 4;   // Fx, Fz, My, Mz
    static constexpr int N_COLS = 5;
    static constexpr int N_AX = 5;
    static constexpr int N_TILTS = 5;

    enum Row : int { ROW_FX = 0, ROW_FZ = 1, ROW_MY = 2, ROW_MZ = 3 };
    enum Col : int { COL_KS = 0, COL_KDF = 1, COL_KT_L = 2, COL_KT_R = 3, COL_KRD = 4 };
    enum TiltIdx : int { TILT_SGRP = 0, TILT_DF = 1, TILT_TL = 2, TILT_TR = 3, TILT_RD = 4 };

    enum SolverStatus : uint8_t {
        STATUS_OK            = 0x00,
        STATUS_SINGULAR      = 0x01,
        STATUS_MAX_ITER      = 0x02,
        STATUS_BASE_K_CLAMP  = 0x04,
    };

    struct State {
        float base_k[N_AX];
        float tilts[N_TILTS];
        float pitch_rad;
    };
    struct Demand { float fx, fz, my, mz; };
    struct Output {
        float delta_k[N_AX];
        float residual[N_ROWS];
        uint8_t sat_lo;
        uint8_t sat_hi;
        uint8_t status;
    };

    void solve(const State &s, const Demand &d, Output &out);
    void build_matrix(float A_out[N_ROWS * N_COLS], const State &s) const;

    // Calibration accessors (Tuner / log) — for non-hot-path reads only.
    float get_gain(int col) const;   // col = COL_KS..COL_KRD
    float get_x(int col) const;
    float get_kt_y() const { return _kt_y; }
    float get_weight(int row) const;  // row = ROW_FX..ROW_MZ
    float get_damping() const { return _damping; }
    uint8_t get_max_iter() const { return uint8_t(_max_iter); }
    bool log_enabled() const { return _log_en != 0; }
    uint8_t get_log_rate_hz() const { return uint8_t(_log_rate); }

private:
    static AP_MantaShark *_singleton;

    // 16 MSAK_ params (per P8.5 design v3 §3, gpt5 a7f1f91 ACK)
    // index   name           default   role
    //   0   KS_GAIN          4.0       calibration: KS thrust gain
    //   1   LOG_EN           1         runtime: BIN log enable
    //   2   KDF_GAIN         2.0       calibration: KDF thrust gain
    //   3   KT_GAIN          2.0       calibration: KT_L/KT_R thrust gain (共用)
    //   4   KRD_GAIN         2.0       calibration: KRD thrust gain
    //   5   KS_X             0.79      calibration: KS x_m (forward+)
    //   6   KDF_X            0.80      calibration: KDF x_m
    //   7   KT_X             0.05      calibration: KT_L/KT_R x_m (共用)
    //   8   KRD_X           -0.50      calibration: KRD x_m
    //   9   KT_Y             0.38      calibration: KT |y_m| (对称)
    //  10   W_FX             1.0       WLS row weight Fx
    //  11   W_FZ             8.0       WLS row weight Fz
    //  12   W_MY             12.0      WLS row weight My
    //  13   W_MZ             2.0       WLS row weight Mz
    //  14   DAMP             1.0e-4    L2 damping
    //  15   MAX_ITER         8         active-set max iter
    //  16   LOG_RATE         50        log Hz (lua hot path 用此 decimate)
    AP_Float _ks_gain;    AP_Int8  _log_en;
    AP_Float _kdf_gain;   AP_Float _kt_gain;   AP_Float _krd_gain;
    AP_Float _ks_x;       AP_Float _kdf_x;     AP_Float _kt_x;       AP_Float _krd_x;
    AP_Float _kt_y;
    AP_Float _w_fx;       AP_Float _w_fz;      AP_Float _w_my;       AP_Float _w_mz;
    AP_Float _damping;
    AP_Int8  _max_iter;
    AP_Int8  _log_rate;
};

namespace AP {
    AP_MantaShark &mantashark();
};
