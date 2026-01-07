# 感知可视化使用指南

## 功能说明

感知可视化节点会在输入图像上标记人脸检测结果，并根据 FSM 状态使用不同颜色：

| 状态 | 颜色 | 说明 |
|------|------|------|
| IDLE | 🟡 黄色 | IDLE 状态检测到的正脸候选 |
| SEARCH | 🟠 橙色 | SEARCH 状态确认的正脸 |
| TRACKING | 🟢 绿色 | TRACKING 状态跟随的目标 |
| 侧脸 | ⚪ 灰色 | 检测到的侧脸（所有状态） |

## 使用方法

### 方法 1: 使用 Launch 文件（推荐）

Launch 文件已自动包含可视化节点：

```bash
# 启动仿真器和婚礼互动节点（包含可视化）
ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
ros2 launch wedding_interaction search_debug.launch.py enable_visualization:=true
```

### 方法 2: 手动启动可视化节点

```bash
# 终端 1: 启动仿真器和婚礼互动节点
ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
ros2 launch wedding_interaction search_debug.launch.py enable_visualization:=false

# 终端 2: 手动启动可视化节点
ros2 run wedding_interaction perception_visualizer \
    --ros-args \
    -p image_topic:=/camera/head/rgb/image_raw \
    -p output_topic:=/wedding/perception/visualization \
    -p publish_rate:=10.0
```

### 方法 3: 使用 rqt_image_view（ROS2 工具）

```bash
# 安装 rqt_image_view（如果未安装）
sudo apt install ros-humble-rqt-image-view

# 启动 rqt_image_view
rqt_image_view /wedding/perception/visualization
```

### 方法 4: 使用自定义查看工具

```bash
# 使用提供的 Python 脚本
cd /home/lingjing/project/engine_ai/wedding_jeston
source /home/lingjing/project/engine_ai/engineai_ros2_workspace/install/setup.bash
python3 scripts/view_visualization.py
```

## 可视化内容

### 图像标记

1. **人脸检测框**：
   - 根据 FSM 状态使用不同颜色
   - 框的粗细：正脸 3px，侧脸 2px

2. **标签信息**：
   - 每个检测框上方显示状态标签
   - 正脸显示 yaw 角度

3. **状态信息（左上角）**：
   - 当前 FSM 状态
   - 检测到的人脸总数和正脸数量
   - face_detected 标志

## 其他验证方法

### 1. ROS2 Topic 监控

```bash
# 查看 FSM 状态
ros2 topic echo /wedding/fsm/state

# 查看人脸检测结果
ros2 topic echo /wedding/perception/faces_json

# 查看 face_detected 标志
ros2 topic echo /wedding/perception/face_detected

# 查看 LookAt 目标
ros2 topic echo /wedding/motion/look_at
```

### 2. 使用 rqt_graph 查看节点连接

```bash
rqt_graph
```

### 3. 使用 rqt_topic 监控 Topic 频率

```bash
rqt_topic
```

### 4. 使用 rqt_plot 绘制数据曲线

```bash
# 绘制 LookAt 坐标变化
rqt_plot /wedding/motion/look_at/point/x /wedding/motion/look_at/point/y
```

### 5. 使用 rqt_console 查看日志

```bash
rqt_console
```

### 6. 录制和回放数据

```bash
# 录制所有相关 topics
ros2 bag record \
    /wedding/fsm/state \
    /wedding/perception/faces_json \
    /wedding/perception/face_detected \
    /wedding/motion/look_at \
    /camera/head/rgb/image_raw \
    /wedding/perception/visualization

# 回放
ros2 bag play <bag_file>
```

### 7. 使用测试脚本

```bash
# 运行集成测试
python3 scripts/test_idle_search_integration.py

# 运行感知调试
python3 scripts/debug_perception.py
```

## 故障排查

### 问题 1: 看不到可视化图像

**检查**：
```bash
# 检查可视化节点是否运行
ros2 node list | grep visualizer

# 检查可视化 topic 是否发布
ros2 topic hz /wedding/perception/visualization

# 检查图像 topic 是否发布
ros2 topic hz /camera/head/rgb/image_raw
```

### 问题 2: 检测框颜色不对

**检查**：
```bash
# 检查 FSM 状态
ros2 topic echo /wedding/fsm/state

# 检查人脸检测结果
ros2 topic echo /wedding/perception/faces_json
```

### 问题 3: 图像延迟或卡顿

**解决**：
- 降低 `publish_rate` 参数（默认 10.0 Hz）
- 检查系统性能
- 使用 `rqt_image_view` 而不是自定义脚本

## 参数说明

### perception_visualizer 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `image_topic` | `/camera/head/rgb/image_raw` | 输入图像 topic |
| `output_topic` | `/wedding/perception/visualization` | 输出可视化图像 topic |
| `publish_rate` | `10.0` | 发布频率 (Hz) |

## 示例场景

### 场景 1: IDLE 状态

- Panel 在远处（8m+）
- 应该看不到检测框
- 状态显示：`State: IDLE` 或 `State: WeddingStateName.IDLE`

### 场景 2: IDLE → SEARCH

- Panel 移动到 1.5m
- 应该看到黄色框（IDLE 候选）
- 状态切换：`IDLE` → `SEARCH`
- 框颜色变为橙色

### 场景 3: SEARCH → TRACKING

- 保持正脸 2 秒
- 状态切换：`SEARCH` → `TRACKING`
- 框颜色变为绿色

### 场景 4: TRACKING → FAREWELL → IDLE

- Panel 移动到远处
- 检测框消失
- 状态切换：`TRACKING` → `FAREWELL` → `IDLE`

## 性能优化

- 如果图像处理太慢，可以降低 `publish_rate`
- 如果图像太大，可以在感知节点中先缩放
- 使用 `rqt_image_view` 通常比自定义脚本更高效

