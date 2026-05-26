/*
 * P8.5b.3 — AP_MantaShark control allocator (real algorithm, log-only)
 *
 * Ported from MantaShark/tools/allocator_cpp/AP_MantaShark_Allocator.h+cpp.
 * Solver: priority WLS + active-set clipping (parity-tested vs Python
 * prototype tools/allocator_proto/, 1000/1000 random cases, max diff 1.6e-6).
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
    // 5 axes: KS, KDF, KT_L, KT_R, KRD
    // 5 tilts (canonical): SGRP, DF, TL, TR, RD (body deg)
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
        float base_k[N_AX];    // KS, KDF, KT_L, KT_R, KRD
        float tilts[N_TILTS];  // SGRP, DF, TL, TR, RD (body deg)
        float pitch_rad;       // not used in build_matrix (body frame)
    };
    struct Demand { float fx, fz, my, mz; };
    struct Output {
        float delta_k[N_AX];
        float residual[N_ROWS];
        uint8_t sat_lo;
        uint8_t sat_hi;
        uint8_t status;
    };

    // Real allocator solve (priority WLS + active-set). Uses double internally for
    // parity with Python prototype.
    void solve(const State &s, const Demand &d, Output &out);

    // Build A matrix (4 rows × 5 cols row-major) — exposed for debug / Tuner.
    void build_matrix(float A_out[N_ROWS * N_COLS], const State &s) const;

private:
    static AP_MantaShark *_singleton;

    // MSAK_ params (P8.5b.3 — 2 placeholders; full 16 in P8.5b.4)
    AP_Float _ks_gain;
    AP_Int8  _log_en;
};

namespace AP {
    AP_MantaShark &mantashark();
};
