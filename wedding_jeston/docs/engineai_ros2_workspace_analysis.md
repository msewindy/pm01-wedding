# EngineAI ROS2 Workspace 项目分析文档

## 1. 项目概述

**EngineAI ROS2 Workspace** 是一个基于 ROS2 的机器人开发工作空间，为 EngineAI 机器人（如 PM01 人形机器人）提供完整的 ROS2 接口和开发工具。

### 1.1 项目定位
- 提供 ROS2 节点和开发工具
- 支持高层和低层两种开发模式
- 包含 Mujoco 仿真环境
- 实现 sim2sim 仿真验证流程

### 1.2 开发模式
| 模式 | 描述 |
|------|------|
| **高层开发 (High-level)** | 通过发布机体速度命令使用 EngineAI 行走控制器 |
| **低层开发 (Low-level)** | 通过发布关节命令开发自定义控制器 |

### 1.3 相关项目
- RL训练框架：[engineai_gym](https://github.com/engineai-robotics/engineai_gym)
- 固件仓库：[engineai_firmware](https://github.com/engineai-robotics/engineai_firmware)

---

## 2. 系统架构

### 2.1 计算单元
| 计算机名称 | 架构 | 用途 | IP地址 | SSH登录 |
|------------|------|------|--------|---------|
| **NeZha** | X86 | 高频运动控制和数据处理 | 192.168.0.163 | user / 1 |
| **Jetson Orin** | ARM | 嵌入式AI和应用功能 | 192.168.0.162 | ubuntu / ubuntu |

### 2.2 目录结构
```
engineai_ros2_workspace/
├── docs/                        # 文档和图片
├── scripts/                     # 构建和同步脚本
│   ├── build_nodes.sh           # 构建节点脚本
│   ├── sync_src.sh              # 同步代码到板卡
│   ├── check_format.sh          # 格式检查
│   └── install.sh               # 安装脚本
└── src/
    ├── interface_example/       # 接口示例（建议在NeZha运行）
    ├── interface_protocol/      # 接口协议（通用模块）
    ├── simulation/              # 仿真环境（主机运行）
    └── third_party/             # 第三方库
```

---

## 3. 核心模块详解

### 3.1 interface_protocol - 接口协议

#### 3.1.1 话题与服务列表
| 名称 | 类型 | 频率 | 消息类型 | 描述 |
|------|------|------|----------|------|
| `/hardware/gamepad_keys` | 发布 | ≥500Hz | GamepadKeys.msg | 手柄按键反馈 |
| `/hardware/imu_info` | 发布 | ≥500Hz | ImuInfo.msg | IMU传感器数据 |
| `/hardware/joint_state` | 发布 | ≥500Hz | JointState.msg | 关节状态反馈 |
| `/hardware/joint_command` | 订阅 | 0~500Hz | JointCommand.msg | 关节命令订阅 |
| `/motion/motion_state` | 发布 | ≥5Hz | MotionState.msg | 机器人运动状态 |
| `/hardware/motor_debug` | 发布 | ≥50Hz | MotorDebug.msg | 电机温度/扭矩调试 |
| `/motion/body_vel_cmd` | 发布 | ≥5Hz | BodyVelCmd.msg | 机体速度命令 |

#### 3.1.2 速度命令范围
- 线速度 X: [-0.5m/s, +0.5m/s]
- 线速度 Y: [-0.2m/s, +0.2m/s]
- 偏航角速度: [-0.5rad/s, +0.5rad/s]

#### 3.1.3 消息类型文件
```
msg/
├── BodyVelCmd.msg       # 机体速度命令
├── GamepadKeys.msg      # 手柄按键
├── Heartbeat.msg        # 心跳
├── ImuInfo.msg          # IMU信息
├── JointCommand.msg     # 关节命令
├── JointState.msg       # 关节状态
├── MotionState.msg      # 运动状态
├── MotorCommand.msg     # 电机命令
├── MotorDebug.msg       # 电机调试
├── MotorState.msg       # 电机状态
├── ParallelParserType.msg
└── Tts.msg              # 文字转语音

srv/
└── EnableMotor.srv      # 电机使能服务
```

### 3.2 interface_example - 接口示例

#### 3.2.1 示例程序
| 文件 | 功能 |
|------|------|
| `body_velocity_control_example.py` | Python高层速度控制示例 |
| `rl_basic_example.cc` | C++ RL控制器部署示例 |
| `hold_joint_status.cc` | 关节状态保持示例 |
| `joint_test_example.cc` | 关节测试示例 |

#### 3.2.2 RL控制器组件
```
src/
├── components/
│   └── message_handler.cc/.hpp    # ROS2消息处理
├── math/
│   ├── mnn_model.cc/.h            # MNN神经网络模型
│   ├── rotation_matrix.cc/.h      # 旋转矩阵工具
│   └── concatenate_vector.h       # 向量拼接
└── parameter/
    └── rl_basic_param.cc/.h       # RL参数管理
```

#### 3.2.3 配置文件
- `config/pm01/rl_basic/basic/rl_basic_param.yaml`: RL参数配置
- `config/pm01/motor.yaml`: 电机配置
- 策略模型：`pm01_v2_rough_ppo_42obs.mnn`

### 3.3 simulation - Mujoco仿真

#### 3.3.1 仿真组件
| 模块 | 功能 |
|------|------|
| `sim_manager` | 仿真管理器 |
| `ros_interface` | ROS2接口封装 |
| `config_loader` | 配置加载器 |
| `simulate` | Mujoco仿真核心 |

#### 3.3.2 机器人模型
- 配置：`assets/config/pm_v2.yaml`
- URDF：`assets/resource/robot/pm_v2/urdf/`
- 网格：`assets/resource/robot/pm_v2/meshes/` (24个STL关节模型)

#### 3.3.3 关节列表
| 部位 | 关节 |
|------|------|
| **左腿** | HIP_YAW_L, HIP_ROLL_L, HIP_PITCH_L, KNEE_PITCH_L, ANKLE_ROLL_L, ANKLE_PITCH_L |
| **右腿** | HIP_YAW_R, HIP_ROLL_R, HIP_PITCH_R, KNEE_PITCH_R, ANKLE_ROLL_R, ANKLE_PITCH_R |
| **躯干** | TORSO_YAW |
| **左臂** | SHOULDER_PITCH_L, SHOULDER_ROLL_L, SHOULDER_YAW_L, ELBOW_PITCH_L, ELBOW_YAW_L |
| **右臂** | SHOULDER_PITCH_R, SHOULDER_ROLL_R, SHOULDER_YAW_R, ELBOW_PITCH_R, ELBOW_YAW_R |
| **头部** | HEAD_YAW |

### 3.4 third_party - 第三方依赖
- `engineai_robotics_third_party_libs.tar.gz`: 预编译库
- `engineai_robotics_third_party_src.tar.xz`: 源码包
- 安装脚本：`install.sh`, `make.sh`

---

## 4. 有限状态机 (FSM)

### 4.1 状态转换图
```
┌─────────────────────────────────────────────────────────────────┐
│                        Joystick Mode                            │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌──────────────┐ │
│  │ Passive │ →  │PD Stand │ →  │Basic Walk│ →  │ Joint Bridge │ │
│  └─────────┘    └─────────┘    └─────────┘    └──────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                    ↓                              ↓
           High-level Dev                   Low-level Dev
        (Body Velocity CMD)              (Joint Commands)
```

### 4.2 模式切换操作
| 状态转换 | 操作 |
|----------|------|
| → PD Stand | LB + A |
| → Basic Walk | LB + X (从PD Stand) |
| → Joint Bridge | LB + CROSS_Y_LEFT (从PD Stand) |
| → Passive | LB + back |

---

## 5. 开发环境要求

### 5.1 基础环境
| 组件 | 版本要求 |
|------|----------|
| Ubuntu | 22.04 |
| ROS2 | Humble Desktop |
| GCC | ≥ 11 |
| CMake | ≥ 3.22 |
| Python | ≥ 3.10 |

### 5.2 软件依赖安装
```bash
sudo apt update
sudo apt install rsync sshpass openssh-client libglfw3-dev libxinerama-dev libxcursor-dev
sudo apt install ros-dev-tools ros-humble-rmw-cyclonedds-cpp ros-humble-ros-base
```

### 5.3 环境变量配置
```bash
echo -e '\nexport ROS_DOMAIN_ID=69\nexport ROS_LOCALHOST_ONLY=0\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 构建与运行

### 6.1 仿真模式
```bash
# 安装第三方依赖
./src/third_party/install.sh

# 构建仿真节点
./scripts/build_nodes.sh sim
source install/setup.bash

# 启动Mujoco仿真器
ros2 launch mujoco_simulator mujoco_simulator.launch.py
```

> ⚠️ **注意**: 运行仿真时，请断开与物理机器人的连接或设置 `ROS_LOCALHOST_ONLY=1`

### 6.2 真机连接
```bash
# 配置网络（192.168.0.0/24网段）
ping 192.168.0.163  # 测试连接
```

> ⚠️ **警告**: 不要使用USB网卡，可能导致ROS2通信问题

### 6.3 高层开发示例
```bash
# 1. 进入Basic Walk模式（参考FSM）
# 2. 构建并运行示例
./scripts/build_nodes.sh example
source install/setup.bash
python src/interface_example/scripts/body_velocity_control_example.py
```

### 6.4 低层开发示例 (RL部署)
```bash
# 1. 进入PD Stand模式
# 2. 进入Joint Bridge模式 (LB + CROSS_Y_LEFT)
# 3. 验证状态
ros2 topic echo /motion/motion_state  # 确认为 joint_bridge

# 4. 运行RL示例
ros2 launch interface_example rl_basic_example.launch.py
```

### 6.5 在目标板上编译
```bash
# 同步代码到NeZha板
./scripts/sync_src.sh nezha

# SSH登录并编译
ssh user@192.168.0.163
cd ~/source/engineai_workspace
./scripts/build_nodes.sh
```

---

## 7. 数据监控

### 7.1 PlotJuggler
```bash
colcon build --packages-select interface_protocol
source install/setup.bash
ros2 run plotjuggler plotjuggler -n
```
- 默认布局文件：`src/interface_protocol/pm_data_layout.xml`

### 7.2 话题监控
```bash
# 查看运动状态
ros2 topic echo /motion/motion_state

# 查看关节状态
ros2 topic echo /hardware/joint_state

# 查看IMU数据
ros2 topic echo /hardware/imu_info
```

---

## 8. 手柄控制说明

### 8.1 操作流程
1. 从包装箱取出机器人，确保悬挂或平放地面
2. 短按电池按钮，再长按直到4个LED灯亮起
3. 等待1分钟，自动启动完成
4. 按 LB + RB 使能电机
5. 按 LB + A 进入PD站立模式
6. 根据FSM流程图操作其他功能

### 8.2 安全警告
- 确保地面足够平整
- 周围保持安全距离
- 准备紧急停止（紧急停止按钮或进入Passive模式）

---

## 9. 与 engineai_humanoid 的关系

| 特性 | engineai_humanoid | engineai_ros2_workspace |
|------|-------------------|-------------------------|
| 通信框架 | LCM | ROS2 |
| 操作系统 | Ubuntu 20.04 | Ubuntu 22.04 |
| 主要用途 | 底层控制算法开发 | ROS2生态集成开发 |
| 仿真环境 | engineai_legged_gym | Mujoco集成 |
| RL推理 | ONNX Runtime | MNN |
| 开发模式 | 单一控制模式 | 高层/低层双模式 |

---

## 10. 技术特点

1. **ROS2 Humble原生支持**: 完整的ROS2节点和接口
2. **双模式开发**: 灵活的高层/低层开发模式选择
3. **Mujoco仿真集成**: 内置仿真器支持sim2sim验证
4. **CycloneDDS**: 高性能DDS中间件
5. **MNN推理**: 移动端优化的神经网络推理框架
6. **跨平台部署**: 支持X86(NeZha)和ARM(Jetson Orin)
7. **PlotJuggler集成**: 实时数据可视化和调试

---

## 11. 常见问题

### Q1: 仿真时如何避免连接到物理机器人？
设置环境变量：`export ROS_LOCALHOST_ONLY=1`

### Q2: 机器人在仿真中摔倒怎么办？
按仿真器中的重置按钮

### Q3: ROS2通信不稳定？
- 避免使用USB网卡
- 检查 ROS_DOMAIN_ID 设置
- 确认 CycloneDDS 正确配置

---

## 12. 文档更新记录

| 版本 | 日期 | 描述 |
|------|------|------|
| 1.0 | 2026-01-01 | 初始版本 |

