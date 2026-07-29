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
#include <AC_PID/AC_PID.h>
#include <Filter/LowPassFilter.h>

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
    // log_rate sanitized [1, 100] Hz — lua hot path 用 (decimation period)
    uint8_t get_log_rate_hz() const {
        int v = (int)_log_rate;
        if (v < 1) v = 1;
        else if (v > 100) v = 100;
        return (uint8_t)v;
    }

    // ───── P8.x 水翼定高控制器 (shadow: 算+log, 不接舵机) ─────
    // 复用 AC_PID 库类, 自喂滤波后的下视测距 (不喂 EKF, 避开浪耦合).
    // 调用方按固定 dt tick (lua 绑定 mantashark_foil:update(dt) 或 Plane 调度).
    // 输出 get_foil_collective() ∈ [-1,1] = 四角 collective 升力需求; 由调用方混进襟翼.
    void  update_foil_height(float dt);
    float get_foil_collective() const { return _foil_coll; }
    bool  foil_height_valid()  const { return _foil_valid; }
    float get_foil_height_filt() const { return _ht_filt; }
    bool  foil_enabled() const { return _foil_en != 0; }

private:
    static AP_MantaShark *_singleton;

    // 17 registered MSAK_ params (indices 0..16). MSAK_CTRL_EN intentionally
    // absent until P8.7+ (gpt5 P8.5b.4 review cdaeaf6).
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

    // ───── 水翼定高控制器 (idx 17 子组 + 18..20) ─────
    AC_PID             _foil_ht_pid;   // MSAK_FH_*  (P/I/D/FF/IMAX/FLTT/FLTE/FLTD/SMAX...)
    LowPassFilterFloat _ht_lpf;        // 下视测距低通 (避浪耦合)
    AP_Int8  _foil_en;                 // MSAK_FOIL_EN  默认 0 (shadow off)
    AP_Float _foil_tgt;                // MSAK_FOIL_TGT 目标离水高度 m
    AP_Float _foil_lpf;                // MSAK_FOIL_FLT 测距低通截止 Hz (兜底, 互补滤波后弃用)
    AP_Float _foil_tilt;               // MSAK_FOIL_TLT 安装基准俯仰角 deg (cos 姿态补偿)
    AP_Float _foil_tau;                // MSAK_FOIL_TAU 互补滤波时间常数 s (超声↔IMU 交叉)
    AP_Float _foil_gate;               // MSAK_FOIL_GATE innovation 门限 m (跳变/混叠样本剔除, 0=关)
    AP_Float _foil_trim;               // MSAK_FOIL_TRM collective 正 trim (托重基准, PID 在其上调制)
    AP_Float _foil_neg;                // MSAK_FOIL_NEG collective 负下限 (默认0 不主动下压, 下沉靠重力)
    float    _foil_coll  = 0.0f;       // collective 输出 [-1,1]
    bool     _foil_valid = false;      // 有效测距 + 在跑
    float    _ht_filt    = 0.0f;       // 融合后真实垂直高度 m (= _ht_fused)
    float    _ht_fused   = 0.0f;       // 互补滤波状态 (超声 LF + IMU 垂速 HF)
    bool     _ht_init    = false;      // 互补滤波是否已初始化 (首个有效样本播种)
    uint16_t _gate_rej   = 0;          // innovation gate 连续拒收计数 (IMU 滑行中)
    float    _ht_rest    = 0.0f;       // 安装0位: disarmed 静置雷达离水距离 (解锁冻结为基准)
    bool     _rest_valid = false;      // 已捕获过有效0位
};

namespace AP {
    AP_MantaShark &mantashark();
};
