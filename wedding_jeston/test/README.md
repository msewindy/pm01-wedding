# 测试目录结构

本目录包含所有测试脚本，按功能分类组织。

## 目录结构

```
test/
├── unit/              # 单元测试
│   ├── test_enums.py                    # 枚举类型测试
│   ├── test_fsm_data.py                 # FSM数据模型测试
│   ├── test_fsm.py                      # FSM核心逻辑测试
│   ├── test_states.py                   # 状态基类测试
│   └── test_idle_search_tracking_logic.py  # IDLE/SEARCH/TRACKING逻辑测试
│
├── integration/       # 集成测试
│   ├── test_idle_search_integration.py  # IDLE→SEARCH集成测试
│   └── test_fsm_standalone.py           # FSM独立集成测试
│
├── perception/        # 感知模块测试
│   ├── test_perception.py               # 感知模块主测试
│   ├── test_image_face_detection.py     # 图像人脸检测测试
│   ├── test_sim_face_detection.py       # 仿真环境人脸检测测试
│   └── test_min_face_size.py            # 最小人脸尺寸测试
│
├── states/            # 状态测试
│   └── test_search_state.py             # SEARCH状态独立测试
│
├── debug/             # 调试工具
│   ├── debug_perception.py              # 感知模块调试工具
│   └── debug_false_detection.py         # 误检测调试工具
│
└── utils/             # 测试工具
    └── mock_perception.py               # 感知数据Mock工具
```

## 运行测试

### 单元测试

```bash
# 运行所有单元测试
pytest test/unit/ -v

# 运行特定测试
pytest test/unit/test_idle_search_tracking_logic.py -v
```

### 集成测试

```bash
# 需要先启动ROS2节点
ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
ros2 launch wedding_interaction search_debug.launch.py

# 运行集成测试
python3 test/integration/test_idle_search_integration.py
```

### 感知测试

```bash
# 使用USB摄像头测试
python3 test/perception/test_perception.py

# 测试图片人脸检测
python3 test/perception/test_image_face_detection.py <图片路径>

# 测试仿真环境人脸检测
python3 test/perception/test_sim_face_detection.py
```

### 状态测试

```bash
# SEARCH状态独立测试
python3 test/states/test_search_state.py
```

### 调试工具

```bash
# 调试感知模块
python3 test/debug/debug_perception.py

# 调试误检测
python3 test/debug/debug_false_detection.py
```

### 测试工具

```bash
# Mock感知数据
python3 test/utils/mock_perception.py --faces 3 --rate 30
```

## 测试分类说明

### 单元测试 (unit/)
- 测试各个模块的独立功能
- 不依赖ROS2或外部服务
- 使用mock对象隔离依赖

### 集成测试 (integration/)
- 测试多个模块或状态的集成功能
- 可能需要ROS2环境
- 测试完整的工作流程

### 感知测试 (perception/)
- 测试人脸检测、跟踪等感知功能
- 可能需要摄像头或图像文件
- 验证MediaPipe等检测器的效果

### 状态测试 (states/)
- 测试FSM各个状态的独立功能
- 验证状态转换逻辑
- 测试状态特定的行为

### 调试工具 (debug/)
- 用于调试和诊断问题的工具脚本
- 帮助定位问题根源
- 提供详细的诊断信息

### 测试工具 (utils/)
- 测试辅助工具
- Mock对象、fixture等
- 可复用的测试组件

