
# 动作与姿态测试指南 (Motion & Pose Verification Guide)

我已经添加了您要求的姿态和动作。以下是在机器人上进行测试的方法。

## 0. 准备工作 (Preparation)
在测试前，请确保重新编译工作空间以应用 Python 代码的更改（特别是如果您没有使用 symlink-install）：

```bash
cd /home/lfw/Desktop/ZQ_develop/pm01-wedding
colcon build --packages-select wedding_interaction
source install/setup.bash
```
重启相关节点：
```bash
# 重启相关节点
# 为了避免 FSM 自动逻辑干扰手动测试，建议使用新建的独立测试启动文件：
ros2 launch wedding_interaction motion_test.launch.py
```

## 1. 验证静态姿态 (Verifying Poses)

发布指令到 `/wedding/motion/set_pose` 话题来触发这些静态姿态。

| 姿态名称 (Pose Name) | 描述 (Description) | 测试指令 (Command) |
| :--- | :--- | :--- |
| **welcome** | **欢迎**：双臂张开 (Arms open wide) | `ros2 topic pub --once /wedding/motion/set_pose std_msgs/msg/String "data: 'welcome'"` |
| **heart_chest** | **胸前比心**：双手在胸前合成心形 (Hands form a heart near chest) | `ros2 topic pub --once /wedding/motion/set_pose std_msgs/msg/String "data: 'heart_chest'"` |
| **heart_head** | **头顶比心**：双手在头顶合成大心形 (Big heart over head) | `ros2 topic pub --once /wedding/motion/set_pose std_msgs/msg/String "data: 'heart_head'"` |
| **akimbo** | **叉腰**：双手叉腰 (Hands on hips) | `ros2 topic pub --once /wedding/motion/set_pose std_msgs/msg/String "data: 'akimbo'"` |

## 2. 验证动态动作 (Verifying Motions)

发布指令到 `/wedding/motion/set_motion` 话题来触发这些持续动作。

| 动作名称 (Motion Name) | 描述 (Description) | 测试指令 (Command) |
| :--- | :--- | :--- |
| **breath** | **呼吸**：肩膀/胸部微幅起伏 (Subtle shoulder/chest breathing) | `ros2 topic pub --once /wedding/motion/set_motion std_msgs/msg/String "data: 'breath'"` |
| **wave_double** | **双手挥手**：双手同时挥动 (Waving with both hands) | `ros2 topic pub --once /wedding/motion/set_motion std_msgs/msg/String "data: 'wave_double'"` |
| **scan_fast** | **快速摇头**：表示“不”或快速浏览 (Quick head shake "No") | `ros2 topic pub --once /wedding/motion/set_motion std_msgs/msg/String "data: 'scan_fast'"` |

> [!TIP]
> **停止动作 / Stop Motion**
> 若要停止当前动作并恢复默认状态，请发送：
> `ros2 topic pub --once /wedding/motion/set_motion std_msgs/msg/String "data: 'stop'"`

## 3. 实现细节 (Implementation Details)

- **运动适配器 (Motion Adapter)**: 升级了代码，支持 `multi_sine` 模式，允许通过配置文件定义通用的多关节正弦波运动。
- **资源文件 (Resources)**: 在 `motion_resources.py` 中添加了上述动作的具体关节角度和波形参数。
