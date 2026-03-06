# 四轴飞行器飞控软件 — 架构设计文档

## 1. 项目概述

开发一个模块化、可移植的四轴飞行器飞控软件，具备以下特性：

- **C++ 核心飞控**：姿态稳定 + 定高控制
- **硬件抽象层（HAL）**：解耦硬件依赖，支持未来适配任意 MCU 平台
- **SITL 仿真**：纯数学物理模型，无需硬件即可测试全部控制逻辑
- **Python 仿真测试与可视化**：自动化测试场景 + 飞行数据绘图
- **CI/CD**：GitHub Actions 自动构建 + 仿真回归测试

---

## 2. 系统架构

```
                     ┌─────────────────────────────────────┐
                     │          Python 仿真测试层           │
                     │  场景脚本 / 数据可视化 / 自动化验证   │
                     └──────────────┬──────────────────────┘
                                    │ subprocess / socket
                     ┌──────────────▼──────────────────────┐
                     │         SITL 可执行文件              │
                     │    飞控核心代码 + SITL-HAL 后端       │
                     └──────────────┬──────────────────────┘
                                    │
          ┌─────────────────────────▼─────────────────────────┐
          │                   飞控核心层                       │
          │  ┌───────────┐ ┌──────────┐ ┌──────────────────┐  │
          │  │ 姿态估计   │ │ PID 控制 │ │ 电机混控 Mixer   │  │
          │  │ AHRS      │ │ 级联PID  │ │ X/+ 模式映射     │  │
          │  └─────┬─────┘ └────┬─────┘ └───────┬──────────┘  │
          │        │            │                │             │
          │  ┌─────▼────────────▼────────────────▼──────────┐  │
          │  │            硬件抽象层 HAL                     │  │
          │  │  IMU / Baro / Motor / Timer / Logger 接口     │  │
          │  └──────────────────┬────────────────────────────┘  │
          └─────────────────────┼───────────────────────────────┘
                                │
              ┌─────────────────┼────────────────────┐
              ▼                 ▼                    ▼
     ┌────────────┐   ┌─────────────────┐   ┌──────────────┐
     │  SITL 后端  │   │ STM32 HAL 后端  │   │ ESP32 后端   │
     │ 物理模型仿真│   │ 未来实现        │   │ 未来实现     │
     └────────────┘   └─────────────────┘   └──────────────┘
```

---

## 3. 目录结构

```
QuadCopter/
├── CMakeLists.txt                  # 顶层 CMake
├── README.md
├── plans/                          # 设计文档
│   └── architecture.md
├── src/
│   ├── CMakeLists.txt
│   ├── core/                       # 飞控核心算法（平台无关）
│   │   ├── CMakeLists.txt
│   │   ├── pid_controller.h        # PID 控制器
│   │   ├── pid_controller.cpp
│   │   ├── attitude_controller.h   # 姿态控制器（级联PID）
│   │   ├── attitude_controller.cpp
│   │   ├── altitude_controller.h   # 定高控制器
│   │   ├── altitude_controller.cpp
│   │   ├── ahrs.h                  # 姿态估计（互补滤波）
│   │   ├── ahrs.cpp
│   │   ├── motor_mixer.h           # 电机混控
│   │   ├── motor_mixer.cpp
│   │   ├── flight_controller.h     # 飞控主循环
│   │   └── flight_controller.cpp
│   ├── hal/                        # 硬件抽象层接口
│   │   ├── CMakeLists.txt
│   │   ├── hal_imu.h               # IMU 接口
│   │   ├── hal_barometer.h         # 气压计接口
│   │   ├── hal_motor.h             # 电机输出接口
│   │   ├── hal_timer.h             # 定时器接口
│   │   ├── hal_logger.h            # 日志接口
│   │   └── hal.h                   # HAL 汇总头文件
│   └── sitl/                       # SITL 仿真后端
│       ├── CMakeLists.txt
│       ├── sitl_imu.h              # SITL IMU 实现
│       ├── sitl_imu.cpp
│       ├── sitl_barometer.h        # SITL 气压计实现
│       ├── sitl_barometer.cpp
│       ├── sitl_motor.h            # SITL 电机实现
│       ├── sitl_motor.cpp
│       ├── sitl_timer.h            # SITL 定时器实现
│       ├── sitl_timer.cpp
│       ├── sitl_logger.h           # SITL 日志实现
│       ├── sitl_logger.cpp
│       ├── physics_model.h         # 四旋翼物理动力学模型
│       ├── physics_model.cpp
│       ├── sitl_main.cpp           # SITL 主程序入口
│       └── （通信通过 sitl_main 的 stdin/stdout 文本协议）
├── tests/
│   ├── CMakeLists.txt
│   ├── test_pid.cpp                # PID 控制器单元测试
│   ├── test_ahrs.cpp               # 姿态估计单元测试
│   ├── test_motor_mixer.cpp        # 电机混控单元测试
│   ├── test_attitude_ctrl.cpp      # 姿态控制集成测试
│   └── test_altitude_ctrl.cpp      # 定高控制集成测试
├── sim/                            # Python 仿真测试
│   ├── requirements.txt            # numpy, matplotlib
│   ├── run_sitl.py                 # SITL 启动与通信管理
│   ├── test_hover.py               # 悬停测试场景
│   ├── test_attitude.py            # 姿态响应测试场景
│   └── plot_results.py             # 数据可视化工具
├── .github/
│   └── workflows/
│       └── ci.yml                  # GitHub Actions CI 配置
└── .gitignore
```

---

## 4. 核心模块设计

### 4.1 硬件抽象层 HAL

HAL 使用纯虚基类定义接口，SITL 和未来的硬件平台各自提供实现：

```cpp
// hal_imu.h
class HalIMU {
public:
    virtual ~HalIMU() = default;
    
    struct IMUData {
        float gyro_x, gyro_y, gyro_z;       // rad/s
        float accel_x, accel_y, accel_z;     // m/s^2
    };
    
    virtual bool init() = 0;
    virtual IMUData read() = 0;
};

// hal_barometer.h
class HalBarometer {
public:
    virtual ~HalBarometer() = default;
    
    struct BaroData {
        float pressure_pa;       // 气压 Pascal
        float temperature_c;     // 温度 摄氏度
        float altitude_m;        // 估算高度 m
    };
    
    virtual bool init() = 0;
    virtual BaroData read() = 0;
};

// hal_motor.h
class HalMotor {
public:
    virtual ~HalMotor() = default;
    
    virtual bool init() = 0;
    // throttle: 0.0 ~ 1.0
    virtual void set_output(int motor_id, float throttle) = 0;
    virtual void arm() = 0;
    virtual void disarm() = 0;
};

// hal_timer.h
class HalTimer {
public:
    virtual ~HalTimer() = default;
    
    virtual uint64_t micros() = 0;     // 微秒时间戳
    virtual uint64_t millis() = 0;     // 毫秒时间戳
    virtual void delay_us(uint32_t us) = 0;
};
```

### 4.2 姿态估计 AHRS

使用互补滤波器融合加速度计和陀螺仪数据：

```
角度估计 = alpha * (上次角度 + 陀螺仪角速度 * dt) + (1 - alpha) * 加速度计角度
```

- alpha 典型值: 0.98
- 陀螺仪: 短期精确，长期漂移
- 加速度计: 长期准确，短期噪声大
- 输出: roll, pitch, yaw（欧拉角）

### 4.3 PID 控制器

通用 PID 控制器，支持：
- 比例/积分/微分增益可调
- 积分抗饱和（anti-windup）
- 微分项低通滤波
- 输出范围限幅

```cpp
class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float output_min, float output_max,
                  float integral_max);
    
    float compute(float setpoint, float measurement, float dt);
    void reset();
    void set_gains(float kp, float ki, float kd);
    
private:
    float kp_, ki_, kd_;
    float output_min_, output_max_;
    float integral_max_;
    float integral_;
    float prev_error_;
    bool first_run_;
};
```

### 4.4 姿态控制器（级联 PID）

```
                     设定角度
                        │
                   ┌────▼────┐
                   │ 角度PID  │ ──► 期望角速度
                   └────┬────┘
                        │
                   ┌────▼────┐
                   │角速度PID │ ──► 控制力矩输出
                   └─────────┘
```

- 外环: 角度 PID（输入：目标角度 vs 当前角度，输出：期望角速度）
- 内环: 角速度 PID（输入：期望角速度 vs 当前角速度，输出：力矩控制量）
- roll, pitch, yaw 各一组级联 PID

### 4.5 定高控制器

```
                     设定高度
                        │
                   ┌────▼────┐
                   │ 高度PID  │ ──► 期望垂直速度
                   └────┬────┘
                        │
                   ┌────▼────┐
                   │ 速度PID  │ ──► 油门增量
                   └─────────┘
```

- 气压计估算当前高度
- 外环: 位置 PID，内环: 速度 PID
- 输出叠加在悬停油门基准值上

### 4.6 电机混控器 Motor Mixer

X 型布局四旋翼电机分配：

```
      前方
   M1    M2       M1: 左前（CCW）
     \  /         M2: 右前（CW）
      \/          M3: 左后（CW）
      /\          M4: 右后（CCW）
     /  \
   M3    M4
```

混控矩阵（X 模式）：

```
M1 = throttle + roll_out - pitch_out - yaw_out
M2 = throttle - roll_out - pitch_out + yaw_out
M3 = throttle + roll_out + pitch_out + yaw_out
M4 = throttle - roll_out + pitch_out - yaw_out
```

输出范围 clamp 到 [0.0, 1.0]。

### 4.7 飞控主循环

```cpp
void FlightController::loop() {
    // 1. 读取传感器
    auto imu_data = hal_imu_->read();
    auto baro_data = hal_baro_->read();
    
    // 2. 姿态估计
    ahrs_.update(imu_data, dt);
    
    // 3. 姿态控制
    auto attitude_out = attitude_ctrl_.compute(
        target_roll_, target_pitch_, target_yaw_rate_,
        ahrs_.roll(), ahrs_.pitch(), ahrs_.yaw_rate(),
        dt
    );
    
    // 4. 定高控制
    float throttle = altitude_ctrl_.compute(
        target_altitude_, baro_data.altitude_m, dt
    );
    
    // 5. 电机混控
    auto motors = mixer_.mix(throttle,
        attitude_out.roll, attitude_out.pitch, attitude_out.yaw);
    
    // 6. 输出到电机
    for (int i = 0; i < 4; i++) {
        hal_motor_->set_output(i, motors[i]);
    }
    
    // 7. 日志记录
    hal_logger_->log(...);
}
```

主循环频率:
- 姿态内环: 500 Hz（2ms）
- 姿态外环: 250 Hz（4ms）
- 定高控制: 50 Hz（20ms）

---

## 5. SITL 仿真设计

### 5.1 四旋翼物理模型

刚体动力学模型，包含：

**状态向量**: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]

- 位置 (x, y, z) 和速度 (vx, vy, vz)：NED 坐标系
- 姿态角 (roll, pitch, yaw)：欧拉角
- 角速度 (p, q, r)：机体坐标系

**动力学方程**:

```
推力:   F_i = k_f * omega_i^2
力矩:   tau_i = k_m * omega_i^2

总推力:  T = sum(F_i)
力矩:   [L, M, N] = mixer_matrix * [F1, F2, F3, F4]

线加速度: a = R * [0, 0, -T/m] + [0, 0, g]
角加速度: [p_dot, q_dot, r_dot] = J^-1 * ([L,M,N] - [p,q,r] x J*[p,q,r])
```

**仿真参数（典型 450mm 机架）**:

| 参数 | 符号 | 典型值 |
|------|------|--------|
| 质量 | m | 1.2 kg |
| 臂长 | L | 0.225 m |
| 转动惯量 Ixx | Jx | 0.0029 kg·m² |
| 转动惯量 Iyy | Jy | 0.0029 kg·m² |
| 转动惯量 Izz | Jz | 0.0055 kg·m² |
| 推力系数 | kf | 1.0e-5 N/(rad/s)² |
| 力矩系数 | km | 1.5e-7 N·m/(rad/s)² |
| 重力加速度 | g | 9.81 m/s² |

**积分方法**: 4 阶 Runge-Kutta（RK4），仿真步长 1ms

### 5.2 传感器仿真

在物理模型真值基础上叠加噪声和偏移：

- **陀螺仪**: 真值 + 高斯噪声(sigma=0.01 rad/s) + 常值偏移
- **加速度计**: 真值 + 高斯噪声(sigma=0.1 m/s²)
- **气压计**: 真值高度 + 高斯噪声(sigma=0.5 m) + 温漂

### 5.3 SITL 执行流程

```
┌───────────────────────────────────────────────┐
│                SITL 主循环                     │
│                                               │
│  1. 飞控主循环执行 → 产生电机输出 [M1..M4]     │
│  2. 物理模型接收电机输出                       │
│  3. 物理模型步进仿真（RK4 积分）               │
│  4. 物理模型产生传感器数据（加噪声）            │
│  5. 传感器数据送回飞控                         │
│  6. 日志输出（CSV / stdout）                   │
│  7. 回到步骤 1                                │
│                                               │
│  ► 通过 stdout/文件输出飞行数据供 Python 分析   │
└───────────────────────────────────────────────┘
```

### 5.4 SITL 与 Python 交互

```
Python 测试脚本
     │
     ├─► 启动 SITL 可执行文件 (subprocess)
     ├─► 通过 stdin 发送指令（设定角度/高度/时长）
     ├─► 从 stdout 或 CSV 文件读取飞行数据
     ├─► 分析数据（超调量、稳态误差、响应时间）
     └─► 生成图表 / 判定 PASS/FAIL
```

通信协议（简易文本行）:

```
# 指令格式（stdin → SITL）
CMD SET_ATTITUDE roll=0.0 pitch=5.0 yaw_rate=0.0
CMD SET_ALTITUDE alt=2.0
CMD ARM
CMD DISARM
CMD RUN duration=10.0

# 数据格式（SITL → stdout，CSV行）
DATA,timestamp_ms,roll,pitch,yaw,alt,vz,m1,m2,m3,m4
```

---

## 6. 控制参数初始值

### 姿态内环 — 角速度 PID

| 轴 | Kp | Ki | Kd | 说明 |
|----|----|----|----|----|
| Roll Rate | 0.15 | 0.05 | 0.002 | 角速度控制 |
| Pitch Rate | 0.15 | 0.05 | 0.002 | 同 Roll |
| Yaw Rate | 0.30 | 0.10 | 0.000 | Yaw 惯量大 |

### 姿态外环 — 角度 P

| 轴 | Kp | 说明 |
|----|----|------|
| Roll Angle | 4.5 | 输出期望角速度 |
| Pitch Angle | 4.5 | 同 Roll |

### 定高

| 环路 | Kp | Ki | Kd |
|------|----|----|-----|
| 高度 P | 1.0 | 0.0 | 0.0 |
| 垂直速度 PID | 0.5 | 0.1 | 0.05 |

---

## 7. 测试策略

### 7.1 单元测试（Google Test）

| 测试项 | 验证内容 |
|--------|---------|
| `test_pid` | PID 输出正确性、anti-windup、reset |
| `test_ahrs` | 互补滤波收敛性、静止状态角度 |
| `test_motor_mixer` | 混控矩阵正确性、输出 clamp |
| `test_attitude_ctrl` | 阶跃响应收敛到设定值 |
| `test_altitude_ctrl` | 定高控制稳定性 |

### 7.2 仿真集成测试（Python）

| 场景 | 验证指标 |
|------|---------|
| 悬停稳定性 | 10s 内姿态偏差 < 1°，高度偏差 < 0.2m |
| 姿态阶跃响应 | 5° roll 阶跃：上升时间 < 0.3s，超调 < 20% |
| 定高阶跃响应 | 1m→2m 高度变化：稳态误差 < 0.1m |
| 扰动恢复 | 外力干扰后 2s 内恢复稳定 |

### 7.3 CI/CD 流水线

```yaml
# GitHub Actions 工作流
触发: push / pull_request
步骤:
  1. CMake 构建（Debug + Release）
  2. 运行 Google Test 单元测试
  3. 编译 SITL 可执行文件
  4. 运行 Python 仿真测试脚本
  5. 上传测试报告和飞行数据图表作为 artifact
```

---

## 8. 技术选型总结

| 项目 | 选择 | 理由 |
|------|------|------|
| 核心语言 | C++17 | 嵌入式兼容、高性能 |
| 构建系统 | CMake 3.16+ | 跨平台、支持交叉编译 |
| 单元测试 | Google Test | 成熟稳定、CI友好 |
| 仿真测试 | Python 3.8+ | 快速原型、强大可视化 |
| 可视化 | matplotlib | 飞行数据绘图 |
| CI/CD | GitHub Actions | 免费、与 GitHub 深度集成 |
| 姿态估计 | 互补滤波 | 实现简单、性能可靠 |
| 控制算法 | 级联 PID | 经典方案、易于调参 |
| 物理模型 | 刚体动力学 + RK4 | 精度足够、实时性好 |

---

## 9. 后续扩展路径

完成基础版本后，可逐步扩展：

1. **EKF 姿态估计** — 替代互补滤波，融合更多传感器
2. **GPS 定点飞行** — 位置环 + 速度环控制
3. **航线规划** — Waypoint 导航
4. **MAVLink 协议** — 与 QGroundControl 等地面站通信
5. **硬件 HAL 后端** — 适配 STM32/ESP32 等具体 MCU
6. **Gazebo 仿真接口** — 高保真 3D 仿真
