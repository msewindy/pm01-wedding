# IDLE 状态仿真调试指南

本文档说明如何在 Mujoco 仿真环境中调试婚礼互动机器人的 IDLE 状态，包括音频准备、参数调整和完整调试流程。

## 前提条件

- Ubuntu 22.04
- ROS2 Humble Desktop
- Python >= 3.10
- 已安装 `engineai_ros2_workspace` 依赖

## 第一步：准备音频文件

IDLE 状态支持播放待机语音，需要先准备音频文件。

### 1.1 安装 TTS 工具（选择其一）

**选项 A：使用 Edge TTS（推荐，音质好）**
```bash
pip install edge-tts
```

**选项 B：使用 Piper TTS（保证声音一致性）**
```bash
pip install piper-tts
# 下载中文模型
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx.json
```

### 1.2 生成音频文件

```bash
cd /home/lingjing/project/engine_ai/wedding_jeston

# 查看所有需要生成的语音列表
python scripts/generate_audio.py --list

# 使用 Edge TTS 生成（推荐）
python scripts/generate_audio.py --engine edge --voice zh-CN-XiaoxiaoNeural

# 或使用 Piper TTS 生成（保证声音一致性）
python scripts/generate_audio.py --engine piper --model /path/to/zh_CN-huayan-medium.onnx

# 只生成 IDLE 相关的语音
python scripts/generate_audio.py --engine edge --only "idle_greeting,idle_welcome,idle_random_1,idle_random_2"
```

生成的音频文件会保存在 `wedding_jeston/audio_resources/` 目录下。

**IDLE 状态相关语音**：
- `idle_greeting.wav` / `idle_greeting.mp3`: "你好呀，欢迎来到婚礼现场！"
- `idle_welcome.wav` / `idle_welcome.mp3`: "欢迎欢迎，今天是个好日子！"
- `idle_random_1.wav` / `idle_random_1.mp3`: "有什么可以帮你的吗？"
- `idle_random_2.wav` / `idle_random_2.mp3`: "需要合影吗？摆个 Pose 就可以啦！"

### 1.3 验证音频文件

```bash
# 检查音频文件是否存在
ls -lh wedding_jeston/audio_resources/idle_*.{wav,mp3}

# 测试播放（需要安装播放器）
# MP3: ffplay -nodisp -autoexit audio_resources/idle_greeting.mp3
# WAV: aplay audio_resources/idle_greeting.wav
```

## 第二步：环境准备

### 2.1 设置环境变量

```bash
# 添加到 ~/.bashrc（如果还没有）
echo 'export ROS_DOMAIN_ID=69' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc  # 重要：防止连接到真实机器人
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

### 2.2 安装第三方依赖（首次）

```bash
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
./src/third_party/install.sh
```

## 第三步：构建代码

### 3.1 构建仿真环境

```bash
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace

# 构建仿真相关包
./scripts/build_nodes.sh sim

# source 环境
source install/setup.bash
```

### 3.2 创建软链接并构建 wedding_interaction

```bash
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace

# 创建软链接（首次）
ln -sf /home/lingjing/project/engine_ai/wedding_jeston src/wedding_jeston

# 构建 wedding_interaction
colcon build --packages-select wedding_interaction

# 重新 source
source install/setup.bash
```

## 第四步：启动调试环境

需要打开 **4 个终端**：

### 终端 1：启动 Mujoco 仿真器

```bash
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash

# 启动仿真器
ros2 launch mujoco_simulator mujoco_simulator.launch.py
```

仿真器窗口应该会显示 PM01 机器人模型。

### 终端 2：启动 FSM、运动适配器和语音播放节点

```bash
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
source install/setup.bash

# 启动 IDLE 调试（使用默认参数）
ros2 launch wedding_interaction idle_debug.launch.py

# 或使用自定义参数
ros2 launch wedding_interaction idle_debug.launch.py \
  fsm_rate:=50.0 \
  control_rate:=500.0
```

你应该看到类似输出：
```
[wedding_fsm_node] WeddingFSMNode initialized
[wedding_fsm_node] FSM rate: 50.0 Hz
[wedding_fsm_node] Entering IDLE state
[motion_adapter_node] MotionAdapterNode initialized at 500.0 Hz
[speech_player_node] SpeechPlayerNode initialized
```

### 终端 3：监控状态和话题

```bash
source /home/lingjing/project/engine_ai/engineai_ros2_workspace/install/setup.bash

# 查看 FSM 状态
ros2 topic echo /wedding/fsm/state

# 查看 look_at 目标（头部/腰部运动）
ros2 topic echo /wedding/motion/look_at

# 查看语音播放状态
ros2 topic echo /wedding/audio/playing
ros2 topic echo /wedding/audio/current

# 查看关节命令（验证运动输出）
ros2 topic echo /hardware/joint_command
```

### 终端 4：发送测试命令

```bash
source /home/lingjing/project/engine_ai/engineai_ros2_workspace/install/setup.bash

# 查看所有 wedding 相关话题
ros2 topic list | grep wedding

# 模拟人脸检测（触发 IDLE -> TRACKING）
ros2 topic pub /wedding/perception/target geometry_msgs/PointStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'}, point: {x: 0.6, y: 0.5, z: 1.5}}" \
  --rate 10

# 停止后回到 IDLE
ros2 topic pub /wedding/fsm/command std_msgs/String "data: 'idle'" --once

# 手动触发语音播放（测试）
ros2 topic pub /wedding/audio/play std_msgs/String "data: 'idle_greeting'" --once
```

## 第五步：验证预期行为

### IDLE 状态行为

在 IDLE 状态下，机器人应该：

1. **头部微动**：头部以约 2.5-3.0 秒周期左右缓慢摆动
2. **腰部微动**：腰部同步微小摆动（幅度约为头部的 50-60%）
3. **动作包切换**：每隔约 10 秒随机切换不同的动作包
4. **语音播放**：有 30% 概率播放待机语音（如"你好呀，欢迎来到婚礼现场！"）
5. **等待目标**：检测到人脸后自动切换到 TRACKING

在仿真器中，你应该能看到机器人头部和腰部的轻微左右摆动。

### 调试检查点

1. **检查 look_at 输出**：
   ```bash
   ros2 topic echo /wedding/motion/look_at
   ```
   - x 值应该在 0.3 ~ 0.7 之间周期变化（取决于动作包幅度）
   - y 值应该保持在 0.5（水平注视）

2. **检查关节命令**：
   ```bash
   ros2 topic echo /hardware/joint_command
   ```
   - `position[12]`（腰部 Yaw）应该有周期性变化
   - `position[23]`（头部 Yaw）应该有周期性变化
   - `stiffness[0-11]`（腿部）应该为 400.0（高刚度保持稳定）

3. **检查状态转换**：
   ```bash
   ros2 topic echo /wedding/fsm/state
   ```
   - 初始应该是 "IDLE"
   - 检测到人脸后应切换到 "TRACKING"

4. **检查语音播放**：
   ```bash
   ros2 topic echo /wedding/audio/playing
   ros2 topic echo /wedding/audio/current
   ```
   - 当播放语音时，`playing` 应为 `true`
   - `current` 应显示当前播放的语音 ID

## 第六步：参数调整

代码支持多种参数调整，可以通过修改代码或 launch 参数来调整行为。

### 6.1 FSM 节点参数

**Launch 参数**（可在启动时调整）：
```bash
ros2 launch wedding_interaction idle_debug.launch.py \
  fsm_rate:=50.0 \          # FSM 运行频率（Hz），默认 50.0
  control_rate:=500.0       # 运动控制频率（Hz），默认 500.0
```

**代码位置**：`wedding_interaction/nodes/wedding_fsm_node.py`
- `fsm_rate`: FSM 主循环频率（默认 50.0 Hz）
- `state_pub_rate`: 状态发布频率（默认 5.0 Hz）

### 6.2 IDLE 状态参数

**代码位置**：`wedding_interaction/fsm/states/idle_state.py`

```python
# 配置参数（类变量，可直接修改）
SWITCH_INTERVAL = 10.0       # 动作包切换间隔（秒），默认 10.0
SPEECH_PROBABILITY = 0.3     # 选择带语音动作包的概率（0.0-1.0），默认 0.3
SPEECH_COOLDOWN = 30.0       # 语音冷却时间（秒），默认 30.0
face_confirm_time = 0.3     # 人脸检测确认时间（秒），默认 0.3
```

**调整方法**：
1. 直接修改 `idle_state.py` 中的类变量
2. 重新构建：`colcon build --packages-select wedding_interaction`
3. 重新 source：`source install/setup.bash`

**示例**：增加语音播放频率
```python
# 在 idle_state.py 中修改
SPEECH_PROBABILITY = 0.5    # 从 0.3 改为 0.5，增加语音概率
SPEECH_COOLDOWN = 20.0      # 从 30.0 改为 20.0，缩短冷却时间
```

### 6.3 动作包参数

**代码位置**：`wedding_interaction/action_pack/action_library.py`

每个动作包包含以下参数：
- `head_amplitude`: 头部摆动幅度（弧度），例如 0.15 = ±8.6°
- `waist_amplitude`: 腰部摆动幅度（弧度），例如 0.08 = ±4.6°
- `period`: 摆动周期（秒），例如 2.5 = 2.5 秒一个周期
- `speech_id`: 关联的语音 ID（如 "idle_greeting"）
- `speech_delay`: 语音延迟播放时间（秒）

**IDLE 动作包示例**：
```python
# 待机摆动 - 中幅度
ActionPack(
    name="idle_sway_medium",
    action_type=ActionType.IDLE_SWAY,
    head_amplitude=0.15,      # ±8.6° 头部
    waist_amplitude=0.08,     # ±4.6° 腰部
    period=2.5,               # 2.5秒周期
    duration=0.0,             # 持续执行
    speech_id=None,           # 无语音
)

# 待机摆动 + 问候语
ActionPack(
    name="idle_sway_with_greeting",
    action_type=ActionType.IDLE_SWAY,
    head_amplitude=0.20,      # ±11.5° 头部
    waist_amplitude=0.10,     # ±5.7° 腰部
    period=2.5,
    duration=0.0,
    speech_id="idle_greeting",
    speech_text="你好呀，欢迎来到婚礼现场！",
    speech_delay=0.5,         # 延迟 0.5 秒播放
)
```

**调整方法**：
1. 修改 `action_library.py` 中的动作包定义
2. 重新构建并 source

**示例**：增大摆动幅度
```python
# 修改 head_amplitude 从 0.15 改为 0.25（±14.3°）
head_amplitude=0.25,
# 修改 period 从 2.5 改为 3.0（更慢的摆动）
period=3.0,
```

### 6.4 运动适配器参数

**Launch 参数**：
```bash
ros2 launch wedding_interaction idle_debug.launch.py \
  control_rate:=500.0 \       # 控制频率（Hz），默认 500.0
  smooth_factor:=0.02         # 平滑系数，默认 0.02（更小=更平滑）
```

**代码位置**：`wedding_interaction/nodes/motion_adapter_node.py`

**可调整参数**：
```python
# 关节限位（弧度）
WAIST_YAW_LIMIT = 0.4  # ±0.4 rad ≈ ±23°（腰部）
HEAD_YAW_LIMIT = 0.6   # ±0.6 rad ≈ ±34°（头部）

# PD 参数
LEG_KP = 400.0   # 腿部刚度（保持稳定）
LEG_KD = 5.0     # 腿部阻尼
BODY_KP = 200.0  # 腰部/头部刚度
BODY_KD = 3.0    # 腰部/头部阻尼
ARM_KP = 100.0   # 手臂刚度
ARM_KD = 2.0     # 手臂阻尼

# 运动系数（在 _on_look_at 方法中）
target_head_offset = x_offset * HEAD_YAW_LIMIT * 0.8   # 80% 限幅
target_waist_offset = x_offset * WAIST_YAW_LIMIT * 0.5  # 50% 限幅
```

**调整方法**：
1. 修改 `motion_adapter_node.py` 中的类变量
2. 或通过 launch 参数调整 `smooth_factor`（无需修改代码）

### 6.5 语音播放节点参数

**Launch 参数**（在 `idle_debug.launch.py` 中）：
```python
{'audio_dir': '/home/lingjing/project/engine_ai/wedding_jeston/audio_resources'},
{'tts_enabled': True},
{'tts_engine': 'mock'},  # 'mock' | 'piper' | 'sherpa'
```

**调整方法**：
1. 修改 `launch/idle_debug.launch.py` 中的参数
2. 或创建新的 launch 文件覆盖参数

**示例**：启用真实 TTS（非 mock）
```python
{'tts_engine': 'piper'},  # 使用 Piper TTS
{'tts_model': '/path/to/zh_CN-huayan-medium.onnx'},  # Piper 模型路径
```

## 常见问题

### Q: 仿真器启动后机器人不动

1. 确认 `motion_adapter_node` 已启动并显示 "Initialization complete"
2. 检查 `/hardware/joint_command` 是否有数据发布
3. 确认 `ROS_LOCALHOST_ONLY=1` 已设置
4. 检查关节状态是否正常接收：`ros2 topic echo /hardware/joint_state`

### Q: 找不到 interface_protocol

确保先构建仿真环境：
```bash
cd /home/lingjing/project/engine_ai/engineai_ros2_workspace
./scripts/build_nodes.sh sim
source install/setup.bash
```

### Q: 状态机不切换

检查感知数据是否正确发布：
```bash
ros2 topic echo /wedding/perception/target
```

### Q: 语音不播放

1. 检查音频文件是否存在：`ls audio_resources/idle_*.{wav,mp3}`
2. 检查 `speech_player_node` 是否启动
3. 检查语音播放话题：`ros2 topic echo /wedding/audio/playing`
4. 确认 `tts_engine` 参数设置正确（仿真模式可用 'mock'）

### Q: 动作幅度太小/太大

调整动作包参数（见 6.3 节）：
- 增大 `head_amplitude` 和 `waist_amplitude` 可增大摆动幅度
- 调整 `period` 可改变摆动速度

### Q: 语音播放太频繁/太少

调整 IDLE 状态参数（见 6.2 节）：
- 增大 `SPEECH_PROBABILITY` 增加语音概率
- 减小 `SPEECH_COOLDOWN` 缩短冷却时间

## 下一步

IDLE 状态调试完成后，可以继续调试：
1. TRACKING 状态（跟随目标）
2. PHOTO_POSING 状态（合影 Pose）
3. 完整状态机流程

