# EngineAI Humanoid 项目分析文档

## 1. 项目概述

**EngineAI Humanoid** 是深圳引擎人工智能机器人技术有限公司开发的双足机器人运动控制算法项目。该项目基于 MIT 的 [Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software) 运动控制算法进行开发。

### 1.1 项目定位
- 主要用于双足人形机器人的运动控制
- 支持强化学习(RL)算法的真实机器人部署
- 提供模型控制(Model-Based Control)与学习控制(Learning-Based Control)相结合的框架

### 1.2 相关项目
- 配套 RL 训练框架：[engineai_legged_gym](https://github.com/engineai-robotics/engineai_legged_gym)
- 用于 sim2sim 仿真验证

---

## 2. 项目架构

### 2.1 目录结构
```
engineai_humanoid/
├── dep-pkgs/                    # 依赖包（电机API等）
│   └── motor-mcu_1.0.8_amd64.deb
├── EngineAI_Controller/         # 核心控制器代码
│   ├── common/                  # 通用算法模块
│   ├── config/                  # 配置文件
│   ├── lcm-types/               # LCM 通信数据类型
│   ├── robot/                   # 机器人硬件抽象层
│   ├── scripts/                 # 运行脚本
│   ├── third-party/             # 第三方库
│   └── user/                    # 用户开发空间
└── README.md
```

### 2.2 核心模块说明

#### 2.2.1 common/ - 通用算法库
| 子模块 | 功能描述 |
|--------|----------|
| Collision/ | 碰撞检测算法 |
| Controllers/ | 控制器基础类 |
| ControlParameters/ | 控制参数管理 |
| Dynamics/ | 动力学计算模块 |
| Math/ | 数学工具库 |
| SimUtilities/ | 仿真工具 |
| Utilities/ | 通用工具类 |

#### 2.2.2 robot/ - 机器人硬件层
- `HardwareBridge`: 硬件桥接接口
- `RobotController`: 机器人控制器基类
- `RobotRunner`: 机器人运行管理
- `SimulationBridge`: 仿真桥接接口
- `rt/`: 实时控制相关

#### 2.2.3 user/EngineAI_Humanoid_Controller/ - 用户控制器
**有限状态机(FSM)状态：**
| 状态 | 功能 |
|------|------|
| FSM_State_Estop | 紧急停止状态 |
| FSM_State_Passive | 被动模式 |
| FSM_State_JointPD | 关节PD控制 |
| FSM_State_LockJoint | 关节锁定 |
| FSM_State_BalanceStand | 平衡站立 |
| FSM_State_RL_Locomotion | RL运动控制 |

**RL控制器模块：**
- `LearningBasedController`: 基于学习的控制器
- `RotationTools`: 旋转工具
- `Lowpassfilter`: 低通滤波器
- 策略文件：`zqsa01_policy.onnx`

#### 2.2.4 third-party/ - 第三方依赖
| 库名 | 用途 |
|------|------|
| Goldfarb_Optimizer | 二次规划优化器 |
| JCQP | 约束QP求解器 |
| qpOASES | QP求解库 |
| osqp | 稀疏QP求解器 |
| ParamHandler | 参数处理（含yaml-cpp） |
| LogitechGamepad | 罗技手柄支持 |
| vectornav | VectorNav IMU驱动 |
| yesense_without_ros | Yesense IMU驱动 |
| SOEM | EtherCAT主站库 |
| inih | INI文件解析 |

#### 2.2.5 lcm-types/ - LCM通信类型
支持多语言绑定（C++, Java, Python）：
- `gamepad_lcmt`: 手柄数据
- `leg_control_command_lcmt`: 腿部控制命令
- `leg_control_data_lcmt`: 腿部控制数据
- `state_estimator_lcmt`: 状态估计数据
- `spi_command_t` / `spi_data_t`: SPI通信数据

---

## 3. 开发环境要求

### 3.1 操作系统
- Ubuntu 20.04

### 3.2 依赖项
| 依赖 | 版本要求 | 说明 |
|------|----------|------|
| CMake | ≥ 3.26 (推荐3.28) | 构建系统 |
| Eigen | 3.3.7 | 矩阵运算库 |
| yaml-cpp | 0.6.3 | YAML解析 |
| LCM | 1.4.0/1.5.0 | 进程间通信 |
| ROS | Noetic | 机器人操作系统 |
| ONNX Runtime | 1.19.0 | 神经网络推理 |
| libmotor | 1.0.8 | 电机API |

---

## 4. 电机API说明

### 4.1 核心函数
| 函数 | 功能 |
|------|------|
| `motor_init()` | 初始化SPI设备 |
| `motor_uninit()` | 释放资源 |
| `get_motor_cmd()` | 获取电机命令 |
| `get_motor_data()` | 获取电机状态 |
| `set_motor_cmd(cmd)` | 发送电机命令 |
| `set_motor_enable_cmd(enable, index)` | 使能/禁用电机 |
| `set_motor_zero_cmd(index)` | 零点校准 |

### 4.2 电机索引
- **左腿** (索引 0-5): 髋关节横滚/偏航/俯仰、膝关节、内踝关节、外踝关节
- **右腿** (索引 6-11): 同上
- **索引 12**: 全部电机

---

## 5. 编译与部署

### 5.1 编译步骤
```bash
git clone https://github.com/engineai-robotics/engineai_humanoid
cd engineai_humanoid/EngineAI_Controller/
mkdir build && cd build
cmake ..
make -j
make install
```

### 5.2 部署架构
- **本地开发设备**: 用于编译生成库和可执行文件
- **主板 NeZha**: 机器人内置计算单元，运行控制程序

### 5.3 网络配置
- 机器人IP: `192.168.0.163`
- 本地静态IP: `192.168.0.100`
- SSH: `ssh user@192.168.0.163` (密码: 1)

---

## 6. 手柄控制

### 6.1 基本操作
| 按键组合 | 功能 |
|----------|------|
| LB + back | 禁用电机（无扭矩） |
| LB + start | 使能电机 |
| LB + B | 进入弯腿站立模式 |
| LB + A | 进入直腿站立模式 |
| LB + X | 进入RL运动模式 |
| A (RL模式下) | 切换站立/行走 |

### 6.2 校准功能
- **欧拉角偏差校准**: B键开启 → 十字键调整 → Y键保存
- **线速度偏差校准**: RB键开启 → 十字键调整 → Y键保存
- **清除偏差**: 校准模式下 X键清零 → Y键保存

### 6.3 标准操作流程
```
开机 → LB+back → LB+start → LB+B/A → 机器人自主站立 → 
LB+X → LB+A → 抬起机器人 → LB+start → LB+back → 关机
```

---

## 7. 代码规范

### 7.1 代码格式化
使用 `.clang-format` 定义的格式：
```bash
sudo apt install clang-format
clang-format -i file.cpp
# 格式化所有源文件
find . -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -style=file -i {} \;
```

### 7.2 提交规范
```
[Fix/New/Modify]: 摘要

1. 提交详细描述

Change-ID
Signed-off-by: xxx <xxx@company.com>
```
- 使用 `git commit -s` 自动生成签名
- 类型：Fix（修复）、New（新功能）、Modify（修改）

---

## 8. 技术特点

1. **MIT Cheetah 架构**: 继承自MIT开源运动控制框架
2. **RL部署支持**: 完整的强化学习策略部署流程
3. **实时控制**: 支持实时内核和高频控制循环
4. **模块化设计**: 清晰的分层架构，便于扩展
5. **多语言LCM支持**: 支持C++/Java/Python的数据记录与调试
6. **硬件抽象**: 统一的硬件接口，支持仿真与真机切换

---

## 9. 文档更新记录

| 版本 | 日期 | 描述 |
|------|------|------|
| 1.0 | 2026-01-01 | 初始版本 |

