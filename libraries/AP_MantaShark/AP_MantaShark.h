/*
 * P8.5b.2 — AP_MantaShark singleton (skeleton, log-only allocator wrapper)
 *
 * Skeleton only. solve() returns dummy zeros. P8.5b.3 ports tools/allocator_cpp.
 *
 * MSAK_ param table (16 registered, P8.4/P8.5 design). MSAK_CTRL_EN 不注册
 * (gpt5 P8.5 v3 review #1) — 控制接管推迟 P8.7+.
 */

#pragma once

#include <AP_Param/AP_Param.h>

class AP_MantaShark {
public:
    AP_MantaShark();

    CLASS_NO_COPY(AP_MantaShark);

    static AP_MantaShark *get_singleton(void) { return _singleton; }

    static const struct AP_Param::GroupInfo var_info[];

    // Allocator solve API (skeleton stub — fills out[] with zeros + STATUS_OK)
    // Mirrors tools/allocator_cpp/AP_MantaShark_Allocator.h signature.
    // 4 demand floats in (fx/fz/my/mz), 5 delta_k + 4 residual + 3 status uints out.
    struct State {
        float base_k[5];     // KS, KDF, KT_L, KT_R, KRD
        float tilts[5];      // SGRP, DF, TL, TR, RD (body deg)
        float pitch_rad;
    };
    struct Demand { float fx, fz, my, mz; };
    struct Output {
        float delta_k[5];
        float residual[4];
        uint8_t sat_lo;
        uint8_t sat_hi;
        uint8_t status;
    };

    void solve(const State &s, const Demand &d, Output &out);

private:
    static AP_MantaShark *_singleton;

    // 16 calibration / runtime param (per P8.5 design §3).
    // First 2 here as smoke test — full 16 in P8.5b.4.
    AP_Float _ks_gain;
    AP_Int8  _log_en;
};

namespace AP {
    AP_MantaShark &mantashark();
};
