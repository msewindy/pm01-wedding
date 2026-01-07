# Wedding Interaction Package

婚礼互动机器人 FSM 实现，基于 ROS2 Humble。

## 功能概述

实现婚礼场景下的机器人互动功能：

- **IDLE（空闲）**: 待机微动，等待目标出现
- **TRACKING（跟随）**: 头部/腰部跟随目标，播放问候语
- **PHOTO_POSING（合影）**: 执行 Pose + 倒数，检测手势触发
- **FAREWELL（送别）**: 挥手 + 送别语
- **SAFE_STOP（安全停止）**: 目标太近时的安全处理

## 目录结构

```
wedding_jeston/
├── wedding_interaction/        # Python 包
│   ├── __init__.py
│   ├── fsm/                   # FSM 核心模块
│   │   ├── __init__.py
│   │   ├── enums.py           # 枚举定义
│   │   ├── wedding_state.py   # 状态基类
│   │   ├── wedding_fsm.py     # FSM 控制器
│   │   ├── wedding_fsm_data.py # 共享数据结构
│   │   └── states/            # 具体状态实现
│   │       ├── idle_state.py
│   │       ├── tracking_state.py
│   │       ├── photo_posing_state.py
│   │       ├── farewell_state.py
│   │       └── safe_stop_state.py
│   └── nodes/                 # ROS2 节点
│       ├── wedding_fsm_node.py
│       └── fsm_test_node.py
├── test/                      # 单元测试
│   ├── test_enums.py
│   ├── test_fsm_data.py
│   ├── test_fsm.py
│   └── test_states.py
├── config/                    # 配置文件
│   └── fsm_config.yaml
├── launch/                    # Launch 文件
│   └── wedding_fsm.launch.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 安装

### 依赖

- ROS2 Humble
- Python 3.10+
- numpy

### 构建

```bash
cd ~/ros2_ws
colcon build --packages-select wedding_interaction
source install/setup.bash
```

## 使用

### 启动 FSM 节点

```bash
ros2 launch wedding_interaction wedding_fsm.launch.py
```

### 启动测试节点

在另一个终端：

```bash
ros2 run wedding_interaction fsm_test
```

### ROS2 接口

**订阅话题:**

| 话题 | 类型 | 描述 |
|------|------|------|
| `/wedding/perception/target` | `geometry_msgs/PointStamped` | 目标位置 (x, y 归一化, z 距离) |
| `/wedding/perception/gesture` | `std_msgs/String` | 手势 ("none", "heart", "v") |
| `/wedding/perception/too_close` | `std_msgs/Bool` | 太近标志 |
| `/wedding/fsm/command` | `std_msgs/String` | 外部命令 |

**发布话题:**

| 话题 | 类型 | 描述 |
|------|------|------|
| `/wedding/fsm/state` | `std_msgs/String` | 当前状态 |
| `/wedding/motion/pose` | `std_msgs/String` | 目标 Pose |
| `/wedding/motion/look_at` | `geometry_msgs/PointStamped` | 注视目标 |
| `/wedding/audio/play` | `std_msgs/String` | 语音资源名 |

**服务:**

| 服务 | 类型 | 描述 |
|------|------|------|
| `/wedding/fsm/reset` | `std_srvs/Trigger` | 重置 FSM |
| `/wedding/fsm/stop` | `std_srvs/Trigger` | 停止 FSM |

### 命令示例

```bash
# 发送合影命令
ros2 topic pub /wedding/fsm/command std_msgs/String "data: 'photo'" --once

# 发送停止命令
ros2 topic pub /wedding/fsm/command std_msgs/String "data: 'stop'" --once

# 重置 FSM
ros2 service call /wedding/fsm/reset std_srvs/srv/Trigger

# 查看状态
ros2 topic echo /wedding/fsm/state
```

## 测试

### 运行单元测试

```bash
cd ~/ros2_ws/src/wedding_jeston
pytest test/ -v
```

### 运行特定测试

```bash
pytest test/test_fsm.py -v
pytest test/test_states.py::TestIdleState -v
```

## 状态转换图

```
                    ┌─────────────┐
    ┌──────────────►│    IDLE     │◄──────────────┐
    │               └──────┬──────┘               │
    │                      │ face_detected        │
    │                      ▼                      │
    │               ┌─────────────┐               │
    │   ┌──────────►│  TRACKING   │───────────┐   │
    │   │           └──────┬──────┘           │   │
    │   │                  │                  │   │
    │   │     gesture      │   face_lost      │   │
    │   │      ┌───────────┴───────────┐      │   │
    │   │      ▼                       ▼      │   │
    │   │ ┌─────────────┐       ┌─────────────┐   │
    │   │ │PHOTO_POSING │       │  FAREWELL   │───┘
    │   │ └──────┬──────┘       └─────────────┘
    │   │        │ complete
    │   └────────┘
    │
    │   too_close (any state)
    │        │
    │        ▼
    │   ┌─────────────┐
    └───│ SAFE_STOP   │
        └─────────────┘
```

## 配置参数

参见 `config/fsm_config.yaml` 了解可配置参数。

## License

MIT

