# ArduPilot 4.6 — STT 精确制导弹药定制分支

基于 ArduPilot 4.6 的 **Skid-To-Turn (STT) 低成本精确制导弹药** 飞控固件。

弹体构型：十字翼尾翼 + 十字翼主翼，14kg，火箭助推后无动力滑翔，半主动激光末制导。

## 定制内容概览

| 模块 | 路径 | 说明 |
|------|------|------|
| AP_STTSeeker | `libraries/AP_STTSeeker/` | 1.064μm 半主动激光导引头 RS-422 驱动 |
| AP_STTCarrier | `libraries/AP_STTCarrier/` | 载机 RS-485 通信（目标坐标、激光编码、发射指令） |
| AP_STTGuidance | `libraries/AP_STTGuidance/` | 中段 PID + 比例导引(PN) + 卡尔曼滤波 + 滚转 PD |
| ModeSTT | `ArduPlane/mode_stt.cpp` | STT 飞行模式（零滚转约束，Lua 接管舵面） |
| stt_mixer() | `ArduPlane/servos.cpp` | 十字翼 4 舵面混控（惯性→体轴旋转） |
| Lua 绑定 | `bindings.desc` | 3 个单例暴露给 Lua：`stt_seeker` / `stt_carrier` / `stt_guidance` |
| SITL 气动 | `libraries/SITL/SIM_Plane.cpp` | `-cruciform` 帧类型：俯仰/偏航气动对称 |

## C++ 库

### AP_STTSeeker — 导引头驱动

- 协议：RS-422, 115200bps, 20Hz
- 帧格式：`0x55 0xAA [帧ID] [数据] [校验]`
- 输出：俯仰/偏航视线角 (LOS Angle)，捕获状态，盲区标志
- 控制：允许/禁止捕获、激光编码绑定、AGC、门宽设置

### AP_STTCarrier — 载机通信

- 协议：RS-485, 115200bps, 半双工
- 帧格式：`0xEB 0x90 [帧ID] [长度] [数据] [校验]`
- 接收帧：挂装查询 (`0x01`)、目标坐标 (`0x02`)、激光编码 (`0x03`)、发射指令 (`0x04`)
- 自动 ACK 应答

### AP_STTGuidance — 制导算法

全部内部计算使用 `double` 精度（GPS 坐标 float 截断会导致 CEP 从 3.5m 退化到 14m）。

**中段导航**：
- 航向 PID + V² 速度增益缩放
- 两阶段：GLIDE（平飞）→ DIVE（俯冲，导引头搜索距离内）

**末段制导**：
- GPS 惯性 PN（比例导引）+ 卡尔曼滤波估计 LOS 角速率
- 几何前馈 + 导引头角度反馈 + 角速率阻尼
- 全部 15 个末段增益可通过 `STT_G_T_*` 参数运行时调节

**滚转稳定**：PD 控制 + V² 气动增益缩放

### 十字翼混控 (stt_mixer)

```
Lua → Script Motor 2/3/4 (pitch/roll/yaw, ±1)
  → 惯性系→体轴旋转 (当前滚转角 φ)
    → 4 舵面混合:
       左翼 = pitch_body - roll
       右翼 = pitch_body + roll
       上翼 = yaw_body - roll
       下翼 = yaw_body + roll
```

## 参数体系

### Lua 参数表 (`STT_*`, idx 1-10)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `STT_PN_N` | 4.0 | PN 导航比 |
| `STT_PN_ALPHA` | 0.4 | LOS 角速率滤波系数 |
| `STT_MAX_G` | 15.0 | 最大过载 (g) |
| `STT_BOOST_SPD` | 180 | 助推目标速度 (m/s) |
| `STT_BOOST_TMO` | 30 | 助推超时 (s) |
| `STT_CRR_SPD` | 33 | 载机携带速度 (m/s) |
| `STT_GND_ALT` | 584 | 地面高度 MSL (m) |
| `STT_SKR_DIST` | 2500 | 导引头搜索距离 (m) |
| `STT_TGT_DIST` | 5000 | 目标距离 (m) |
| `STT_BOOST_ALT` | 3000 | 助推高度 AGL (m) |

### C++ 制导增益 (`STT_G_*`, idx 1-26)

**中段 (idx 1-5)**：`MID_H_KP/KI/KD`, `MID_P_KP`, `MID_VREF`

**卡尔曼滤波 (idx 6-8)**：`KF_Q`, `KF_R`, `KF_MAXR`

**滚转 (idx 9-10)**：`ROLL_KP`, `ROLL_KD`

**末段参考速度 (idx 11)**：`TRM_VREF`

**末段增益 (idx 12-26)**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `STT_G_T_FF_P1` | 0.30 | 俯仰前馈 (<300m) |
| `STT_G_T_FF_P2` | 0.20 | 俯仰前馈 (300-500m) |
| `STT_G_T_FF_P3` | 0.08 | 俯仰前馈 (>500m) |
| `STT_G_T_RFF_G1` | 0.030 | 速率前馈 (<300m) |
| `STT_G_T_RFF_G2` | 0.012 | 速率前馈 (>=300m) |
| `STT_G_T_YAW_FF` | 0.020 | 偏航前馈增益 |
| `STT_G_T_YAW_MAX` | 0.05 | 偏航指令限幅 |
| `STT_G_T_PN_P` | 0.50 | PN 俯仰增益 |
| `STT_G_T_PN_Y` | 0.0 | PN 偏航增益 |
| `STT_G_T_SKR_P1` | 0.03 | 导引头俯仰 (<300m) |
| `STT_G_T_SKR_P2` | 0.015 | 导引头俯仰 (300-1200m) |
| `STT_G_T_KD_P_CL` | 0.003 | 俯仰阻尼 (<200m) |
| `STT_G_T_KD_P_FR` | 0.010 | 俯仰阻尼 (>=200m) |
| `STT_G_T_KD_Y_CL` | 0.005 | 偏航阻尼 (<300m) |
| `STT_G_T_KD_Y_FR` | 0.015 | 偏航阻尼 (>=300m) |

## SITL 仿真

### 气动模型

`-cruciform` 帧类型强制俯仰/偏航气动对称：
- `c_n_b = |c_m_a| * (c/b)` — 静稳定性对称
- `c_l_b = 0`, `c_l_r = 0` — 消除侧滑-滚转耦合
- `Izz = Iyy` — 惯性矩对称
- SERVO5-8 读取十字翼 4 舵面输出

### 构建与运行

```bash
# 构建
./waf configure --board sitl
./waf build --target bin/arduplane

# SITL 运行 (需配合 STT 仿真框架)
cd STT
python3 sim/auto_sim.py              # 单次仿真
python3 sim/auto_sim.py --batch 10   # 批量统计
```

## 飞行阶段

```
PRE_LAUNCH → BOOST → MIDCOURSE (glide→dive) → TERMINAL → IMPACT
   载机带飞     火箭助推    PID 航线控制         PN 精确制导    触地
```

1. **PRE_LAUNCH**：等待载机 RS-485 发射指令 + 目标坐标
2. **BOOST**：继电器点火，全油门加速，航向初步修正
3. **MIDCOURSE**：无动力滑翔，GPS 航向 PID，卡尔曼滤波预热
4. **TERMINAL**：导引头锁定后切入，GPS-PN + 几何前馈 + 导引头反馈复合制导
5. **IMPACT**：飞越检测（3D 距离连续递增 5 帧）触发

## 硬件平台

- **飞控**：CUAV-X7 (STM32H743)
- **导引头**：1.064μm 半主动激光，RS-422 (SERIAL5)
- **载机接口**：RS-485 (SERIAL6)
- **舵面**：4 × 十字翼舵机 (SERVO5-8)

## 基于 ArduPilot 4.6

本分支基于 [ArduPilot](https://github.com/ArduPilot/ardupilot) 4.6 版本。
上游许可证：GPLv3。详见 [COPYING.txt](COPYING.txt)。
