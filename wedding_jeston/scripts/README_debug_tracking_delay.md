# 跟踪延迟调试脚本运行指南

## 脚本功能

`debug_tracking_delay.py` 用于排查第一阶段跟踪发散的根本原因：
1. 验证机器人头部是否实际转动（运动控制延迟）
2. 验证图像延迟大小（图像延迟）
3. 同步观察look_at、关节位置、图像目标位置的变化

## 必需的ROS2节点

运行此脚本需要以下节点：

### 1. **motion_adapter_node**（必需）
- **功能**: 接收look_at命令，转换为关节控制
- **订阅**: `/wedding/motion/look_at` (PointStamped)
- **发布**: `/hardware/joint_command` (JointCommand)
- **订阅**: `/hardware/joint_state` (JointState) - 用于读取当前关节位置

### 2. **perception_node**（必需）
- **功能**: 人脸检测，发布检测结果
- **订阅**: 图像话题（如 `/camera/head/rgb/image_raw`）
- **发布**: `/wedding/perception/faces_json` (String) - 脚本需要订阅此话题

### 3. **硬件/仿真器**（必需）
- **功能**: 提供关节状态反馈
- **发布**: `/hardware/joint_state` (JointState) - 脚本需要订阅此话题
- **订阅**: `/hardware/joint_command` (JointCommand) - 接收motion_adapter的命令

**注意**: 
- 如果是仿真环境，需要启动 `mujoco_simulator`
- 如果是真实硬件，需要硬件驱动节点

### 4. **图像源**（必需）
- **功能**: 提供相机图像
- **发布**: 图像话题（如 `/camera/head/rgb/image_raw`）
- **说明**: perception_node需要订阅此话题

**注意**:
- 如果是仿真环境，mujoco_simulator会发布图像
- 如果是真实硬件，需要相机驱动节点

## 快速启动方式

### 方式1: 使用现有的launch文件（推荐）

如果使用仿真环境：

```bash
# 终端1: 启动仿真器
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py

# 终端2: 启动必要的节点
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash
ros2 launch wedding_interaction search_debug.launch.py

# 终端3: 运行调试脚本
cd /home/lingjing/project/engine_ai/wedding_jeston
source /opt/ros/humble/setup.bash
source /home/lingjing/project/engine_ai/engineai_ros2_workspace/install/setup.bash
python3 scripts/debug_tracking_delay.py
```

### 方式2: 手动启动各个节点

如果只需要最小化的节点集合：

```bash
# 终端1: 启动仿真器（如果使用仿真）
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py

# 终端2: 启动perception_node
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash
ros2 run wedding_interaction perception_node \
  --ros-args \
  -p image_topic:=/camera/head/rgb/image_raw \
  -p detection_rate:=30.0

# 终端3: 启动motion_adapter_node
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash
ros2 run wedding_interaction motion_adapter \
  --ros-args \
  -p control_rate:=500.0 \
  -p smooth_factor:=0.02

# 终端4: 运行调试脚本
cd /home/lingjing/project/engine_ai/wedding_jeston
source /opt/ros/humble/setup.bash
source /home/lingjing/project/engine_ai/engineai_ros2_workspace/install/setup.bash
python3 scripts/debug_tracking_delay.py
```

## 节点依赖关系图

```
┌─────────────────┐
│  mujoco_simulator│ (或真实硬件)
│                 │
│  发布:          │
│  - /hardware/   │
│    joint_state  │
│  - /camera/head/│
│    rgb/image_raw│
└────────┬────────┘
         │
         │ 订阅图像
         ▼
┌─────────────────┐
│ perception_node │
│                 │
│  发布:          │
│  - /wedding/    │
│    perception/  │
│    faces_json   │
└────────┬────────┘
         │
         │ 订阅faces_json
         ▼
┌─────────────────┐
│debug_tracking_  │
│delay.py         │
│                 │
│  发布:          │
│  - /wedding/    │
│    motion/      │
│    look_at      │
│                 │
│  订阅:          │
│  - /hardware/   │
│    joint_state  │
│  - /wedding/    │
│    perception/  │
│    faces_json   │
└────────┬────────┘
         │
         │ 发布look_at
         ▼
┌─────────────────┐
│motion_adapter_  │
│node              │
│                 │
│  订阅:          │
│  - /wedding/    │
│    motion/      │
│    look_at      │
│  - /hardware/   │
│    joint_state  │
│                 │
│  发布:          │
│  - /hardware/   │
│    joint_command│
└────────┬────────┘
         │
         │ 发布joint_command
         ▼
┌─────────────────┐
│  mujoco_simulator│ (或真实硬件)
│                 │
│  订阅:          │
│  - /hardware/   │
│    joint_command│
└─────────────────┘
```

## 验证节点是否运行

运行以下命令检查节点状态：

```bash
# 检查所有节点
ros2 node list

# 应该看到：
# /motion_adapter_node
# /perception_node
# /tracking_delay_debugger (调试脚本)
# /mujoco_simulator (如果使用仿真)

# 检查话题
ros2 topic list

# 应该看到：
# /wedding/motion/look_at
# /wedding/perception/faces_json
# /hardware/joint_state
# /hardware/joint_command
# /camera/head/rgb/image_raw (或你的图像话题)

# 检查话题是否有数据
ros2 topic echo /wedding/perception/faces_json --once
ros2 topic echo /hardware/joint_state --once
```

## 常见问题

### 1. 脚本提示 "Warning: interface_protocol not found"
- **原因**: 无法导入 `interface_protocol.msg.JointState`
- **解决**: 确保已构建并source了 `engineai_ros2_workspace`
- **影响**: 脚本仍可运行，但无法监控关节状态

### 2. 没有收到图像数据
- **检查**: `ros2 topic echo /wedding/perception/faces_json`
- **可能原因**: 
  - perception_node没有运行
  - 图像话题名称不匹配
  - 相机没有图像输出

### 3. 没有收到关节状态
- **检查**: `ros2 topic echo /hardware/joint_state`
- **可能原因**:
  - 仿真器/硬件没有运行
  - 话题名称不匹配

### 4. look_at命令没有效果
- **检查**: `ros2 topic echo /wedding/motion/look_at`
- **可能原因**:
  - motion_adapter_node没有运行
  - 话题名称不匹配

## 测试流程

1. **启动所有必需节点**（见上方）
2. **等待2-3秒**，让系统稳定
3. **运行调试脚本**:
   ```bash
   python3 scripts/debug_tracking_delay.py
   ```
4. **观察输出**:
   - 脚本会在2秒后自动开始测试
   - 设置 `look_at=0.3`
   - 记录关节和图像的变化时间
   - 10秒后输出总结
5. **分析结果**:
   - 如果关节变化快(<0.1s)，图像变化慢(>0.15s) → 图像延迟问题
   - 如果关节不变化或变化很慢(>0.15s) → 运动控制延迟问题

## 注意事项

1. **不需要启动FSM节点**: 调试脚本直接发布look_at命令，不需要FSM
2. **需要有人脸在视野中**: 脚本需要检测到人脸才能分析图像变化
3. **仿真环境**: 如果使用仿真，确保仿真器中有人脸模型或目标
4. **真实硬件**: 如果使用真实硬件，确保相机能看到人脸

