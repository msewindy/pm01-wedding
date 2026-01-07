# Interview 测试快速开始指南

## 快速测试步骤

### 1. 环境准备

```bash
# 检查依赖
python3 -c "from wedding_interaction.fsm.states.interview_recorder import InterviewRecorder; print('Dependencies:', InterviewRecorder.check_dependencies())"
```

### 2. 单元测试（无需ROS2）

```bash
# 运行InterviewRecorder单元测试
cd wedding_jeston
pytest test/unit/test_interview_recorder.py -v

# 运行所有Interview相关单元测试
pytest test/unit/ -k interview -v
```

### 3. 集成测试（需要ROS2环境）

#### 3.1 启动测试环境

**终端1**: 启动仿真器
```bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
```

**终端2**: 启动Interview测试节点
```bash
ros2 launch wedding_interaction interview_test.launch.py
```

#### 3.2 运行集成测试

**终端3**: 运行测试脚本
```bash
cd wedding_jeston
python3 test/functional/test_interview_manual.py
```

或者手动触发：
```bash
# 触发采访
ros2 topic pub /wedding/fsm/command std_msgs/String "data: 'interview'" --once

# 监控状态
ros2 topic echo /wedding/fsm/state

# 监控语音
ros2 topic echo /wedding/audio/play

# 监控Pose
ros2 topic echo /wedding/motion/pose
```

### 4. 验证录制文件

```bash
# 检查录制文件
ls -lh /tmp/wedding_blessings/

# 播放测试（如果安装了ffplay）
ffplay /tmp/wedding_blessings/blessing_*.mp4
```

## 测试检查清单

### 基本功能
- [ ] 单元测试通过
- [ ] 状态转换正常（TRACKING → INTERVIEW）
- [ ] 录制启动正常
- [ ] 录制文件生成
- [ ] 状态转换正常（INTERVIEW → TRACKING）

### 录制功能
- [ ] 视频录制正常
- [ ] 音频录制正常
- [ ] 文件格式正确（MP4）
- [ ] 文件大小合理

### 同步验证
- [ ] 播放视频检查音视频同步
- [ ] 无明显延迟（< 100ms）

### 错误处理
- [ ] 无设备时优雅失败
- [ ] 资源清理正常
- [ ] 无内存泄漏

## 常见问题

### 1. 依赖缺失

```bash
# 安装OpenCV
pip install opencv-python

# 安装PyAudio
sudo apt install portaudio19-dev python3-pyaudio

# 安装ffmpeg
sudo apt install ffmpeg
```

### 2. 权限问题

```bash
# 确保有摄像头和麦克风权限
# 检查设备
ls -l /dev/video*
arecord -l  # 列出音频设备
```

### 3. 录制文件未生成

- 检查保存路径权限：`/tmp/wedding_blessings/`
- 检查日志中的错误信息
- 确认依赖都已安装

### 4. 音视频不同步

- 检查启动时间差日志
- 验证ffmpeg合并命令
- 检查帧率是否稳定

## 详细测试文档

参考：`wedding_jeston/docs/INTERVIEW_TEST_PLAN.md`

