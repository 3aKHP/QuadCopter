# QuadCopter Flight Controller

一个模块化、可移植的四轴飞行器飞控软件，支持 SITL（Software-In-The-Loop）仿真测试。

## 特性

- **C++17 核心飞控算法** — 姿态稳定 + 定高控制
- **硬件抽象层 (HAL)** — 解耦硬件依赖，支持多平台适配
- **SITL 仿真** — 纯数学物理模型，无需硬件即可测试
- **Python 测试与可视化** — 自动化仿真测试场景
- **CI/CD** — GitHub Actions 自动构建和回归测试

## 快速开始

### 前置要求

- CMake 3.16+
- C++17 兼容编译器 (GCC 7+, Clang 5+, MSVC 2017+)
- Python 3.8+ (仿真测试)
- Git (用于下载 Google Test)

### 构建

```bash
# 配置
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON -DBUILD_SITL=ON

# 编译
cmake --build build --config Debug

# 运行单元测试
ctest --test-dir build --output-on-failure -C Debug
```

### 运行 SITL 仿真

```bash
# 安装 Python 依赖
python -m pip install -r sim/requirements.txt

# 运行悬停测试
python sim/test_hover.py

# 运行姿态响应测试
python sim/test_attitude.py
```

### 交互式 SITL

```bash
python sim/run_sitl.py
```

## 项目结构

```
QuadCopter/
├── src/
│   ├── core/          # 飞控核心算法（平台无关）
│   │   ├── pid_controller    # PID 控制器
│   │   ├── ahrs              # 姿态估计（互补滤波）
│   │   ├── attitude_controller # 姿态控制器（级联PID）
│   │   ├── altitude_controller # 定高控制器
│   │   ├── motor_mixer       # 电机混控器
│   │   └── flight_controller # 飞控主循环
│   ├── hal/           # 硬件抽象层接口
│   └── sitl/          # SITL 仿真后端
│       ├── physics_model     # 四旋翼物理模型 (RK4)
│       └── sitl_*           # HAL 接口的仿真实现
├── tests/             # Google Test 单元测试
├── sim/               # Python 仿真测试与可视化
├── plans/             # 设计文档
└── .github/workflows/ # CI/CD 配置
```

## 架构

详见 [plans/architecture.md](plans/architecture.md)

## License

MIT
