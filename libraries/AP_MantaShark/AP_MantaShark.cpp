/*
 * P8.5b.2 — AP_MantaShark singleton (skeleton)
 *
 * Skeleton only. solve() returns zeros. Real allocator ported in P8.5b.3.
 */

#include "AP_MantaShark.h"

AP_MantaShark *AP_MantaShark::_singleton = nullptr;

// First 2 params smoke test (P8.5b.2). Full MSAK_ table populated in P8.5b.4.
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
        // shouldn't happen — singleton constructed once at Plane init
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// P8.5b.2 skeleton: returns zeros + STATUS_OK. No actual computation.
// P8.5b.3 will port tools/allocator_cpp/ here.
void AP_MantaShark::solve(const State &s, const Demand &d, Output &out) {
    (void)s; (void)d;
    for (int i = 0; i < 5; ++i) out.delta_k[i] = 0.0f;
    for (int i = 0; i < 4; ++i) out.residual[i] = 0.0f;
    out.sat_lo = 0;
    out.sat_hi = 0;
    out.status = 0;   // STATUS_OK
}

namespace AP {
    AP_MantaShark &mantashark() {
        return *AP_MantaShark::get_singleton();
    }
}
