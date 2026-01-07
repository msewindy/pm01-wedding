# Interview 音视频采集测试方案

## 测试目标

验证 Interview 功能的完整性和正确性，包括：
1. 音视频录制功能
2. 音视频同步
3. 状态转换逻辑
4. ROS2 集成
5. 错误处理

## 测试层次

### 1. 单元测试（Unit Tests）

#### 1.1 InterviewRecorder 组件测试

**文件**: `test/unit/test_interview_recorder.py`

**测试用例**:

1. **依赖检查测试**
   - `test_check_dependencies()`: 验证依赖检查功能
   - 测试 OpenCV、PyAudio、ffmpeg 的检测

2. **VideoRecorder 测试**
   - `test_video_recorder_init()`: 测试初始化
   - `test_video_recorder_camera_mode()`: 测试摄像头模式
   - `test_video_recorder_callback_mode()`: 测试回调模式
   - `test_video_recorder_frame_rate()`: 测试帧率控制
   - `test_video_recorder_stop()`: 测试停止和清理

3. **AudioRecorder 测试**
   - `test_audio_recorder_init()`: 测试初始化
   - `test_audio_recorder_default_device()`: 测试默认设备检测
   - `test_audio_recorder_record()`: 测试录制功能
   - `test_audio_recorder_stop()`: 测试停止和保存

4. **InterviewRecorder 集成测试**
   - `test_recorder_init()`: 测试初始化
   - `test_recorder_start()`: 测试启动
   - `test_recorder_stop()`: 测试停止
   - `test_recorder_cleanup()`: 测试资源清理

#### 1.2 InterviewState 逻辑测试

**文件**: `test/unit/test_interview_state.py`

**测试用例**:

1. **状态初始化**
   - `test_interview_state_init()`: 测试初始化
   - `test_interview_state_on_enter()`: 测试进入状态

2. **对话流程测试**
   - `test_interview_flow_start()`: 测试开始阶段
   - `test_interview_flow_greeting()`: 测试问候阶段
   - `test_interview_flow_question()`: 测试提问阶段
   - `test_interview_flow_listening()`: 测试监听阶段
   - `test_interview_flow_ending()`: 测试结束阶段

3. **状态转换测试**
   - `test_transition_to_tracking()`: 测试转换到TRACKING
   - `test_transition_on_command()`: 测试命令中断

---

### 2. 集成测试（Integration Tests）

#### 2.1 InterviewRecorder 集成测试

**文件**: `test/integration/test_interview_recorder_integration.py`

**测试场景**:

1. **完整录制流程测试**
   - 启动录制
   - 模拟视频帧输入（使用mock回调）
   - 录制音频（使用真实麦克风或mock）
   - 停止录制
   - 验证输出文件存在
   - 验证文件格式正确

2. **音视频同步测试**
   - 同时启动视频和音频录制
   - 记录启动时间差
   - 验证ffmpeg合并时的补偿
   - 检查最终文件的同步质量

3. **错误处理测试**
   - 测试摄像头不可用
   - 测试麦克风不可用
   - 测试ffmpeg不可用
   - 测试文件写入失败

#### 2.2 InterviewState 与 FSM 集成测试

**文件**: `test/integration/test_interview_state_integration.py`

**测试场景**:

1. **状态转换流程**
   - TRACKING → INTERVIEW（通过命令触发）
   - INTERVIEW 内部阶段转换
   - INTERVIEW → TRACKING（完成）

2. **录制生命周期**
   - 验证 on_enter 时启动录制
   - 验证 run 时录制进行中
   - 验证 on_exit 时停止录制

3. **跟随功能集成**
   - 验证在采访状态下仍能跟随目标
   - 验证目标丢失处理

---

### 3. ROS2 集成测试（ROS2 Integration Tests）

#### 3.1 完整系统测试

**文件**: `test/integration/test_interview_ros2_integration.py`

**测试场景**:

1. **ROS2 Topic 集成**
   - 订阅 `/camera/head/rgb/image_raw`
   - 验证图像转换正确
   - 验证图像存储到 `fsm.data.perception.current_frame`

2. **状态转换触发**
   - 通过 `/wedding/fsm/command` 发送 `CMD_START_INTERVIEW`
   - 验证状态转换到 INTERVIEW
   - 验证录制启动

3. **完整采访流程**
   - 启动仿真器和节点
   - 触发采访命令
   - 等待采访完成
   - 验证录制文件生成

#### 3.2 Launch 文件测试

**文件**: `launch/interview_test.launch.py`

创建专门的测试launch文件，包含：
- Simulation节点
- Perception节点
- FSM节点
- 测试节点

---

### 4. 功能测试（Functional Tests）

#### 4.1 手动测试脚本

**文件**: `test/functional/test_interview_manual.py`

**测试步骤**:

1. **环境准备**
   ```bash
   # 启动仿真器
   ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
   
   # 启动婚礼互动节点
   ros2 launch wedding_interaction interview_test.launch.py
   ```

2. **触发采访**
   ```bash
   # 发送采访命令
   ros2 topic pub /wedding/fsm/command std_msgs/String "data: 'interview'" --once
   ```

3. **观察行为**
   - 检查状态转换日志
   - 检查录制启动日志
   - 观察机器人动作（麦克风pose）
   - 观察对话流程

4. **验证结果**
   - 检查录制文件是否生成
   - 播放视频验证音视频同步
   - 检查文件大小和时长

#### 4.2 自动化功能测试

**文件**: `test/functional/test_interview_automated.py`

使用ROS2节点自动执行测试流程。

---

### 5. 性能测试（Performance Tests）

#### 5.1 录制性能测试

**文件**: `test/performance/test_interview_performance.py`

**测试指标**:

1. **帧率稳定性**
   - 目标：30fps ± 1%
   - 测试：记录实际帧率，计算方差

2. **内存使用**
   - 测试长时间录制（60秒）的内存占用
   - 验证无内存泄漏

3. **CPU使用**
   - 测试录制时的CPU占用率
   - 目标：< 50%

4. **文件大小**
   - 测试不同录制时长的文件大小
   - 验证文件大小合理

#### 5.2 同步精度测试

**文件**: `test/performance/test_av_sync_precision.py`

**测试方法**:

1. **启动时间差测试**
   - 记录视频和音频的启动时间
   - 验证时间差 < 50ms

2. **同步误差测试**
   - 使用测试视频（带时间戳）
   - 录制并分析同步误差
   - 目标：误差 < 100ms

---

### 6. 错误场景测试（Error Scenario Tests）

#### 6.1 异常情况测试

**文件**: `test/error/test_interview_errors.py`

**测试场景**:

1. **依赖缺失**
   - OpenCV不可用
   - PyAudio不可用
   - ffmpeg不可用

2. **设备不可用**
   - 摄像头不可用
   - 麦克风不可用

3. **资源不足**
   - 磁盘空间不足
   - 内存不足

4. **中断场景**
   - 录制过程中状态转换
   - 录制过程中系统关闭

---

## 测试执行计划

### 阶段1：单元测试（开发阶段）

```bash
# 运行InterviewRecorder单元测试
pytest test/unit/test_interview_recorder.py -v

# 运行InterviewState单元测试
pytest test/unit/test_interview_state.py -v
```

**目标**: 确保各个组件功能正确

### 阶段2：集成测试（集成阶段）

```bash
# 运行InterviewRecorder集成测试
pytest test/integration/test_interview_recorder_integration.py -v

# 运行InterviewState集成测试
pytest test/integration/test_interview_state_integration.py -v
```

**目标**: 确保组件间协作正确

### 阶段3：ROS2集成测试（系统测试阶段）

```bash
# 启动测试环境
ros2 launch wedding_interaction interview_test.launch.py

# 运行ROS2集成测试
pytest test/integration/test_interview_ros2_integration.py -v
```

**目标**: 确保ROS2集成正确

### 阶段4：功能测试（验收测试阶段）

```bash
# 手动测试
python3 test/functional/test_interview_manual.py

# 自动化测试
python3 test/functional/test_interview_automated.py
```

**目标**: 验证完整功能

### 阶段5：性能测试（性能验证阶段）

```bash
# 性能测试
pytest test/performance/test_interview_performance.py -v

# 同步精度测试
pytest test/performance/test_av_sync_precision.py -v
```

**目标**: 验证性能指标

---

## 测试数据准备

### 1. Mock数据

- **Mock视频帧**: 生成测试用的OpenCV图像
- **Mock音频数据**: 生成测试用的音频信号
- **Mock ROS2消息**: 模拟ROS2 Image消息

### 2. 测试视频

- 准备带时间戳的测试视频（用于同步验证）
- 准备不同分辨率的测试视频

### 3. 测试音频

- 准备测试音频文件（用于同步验证）
- 准备不同采样率的测试音频

---

## 测试检查清单

### InterviewRecorder 测试

- [ ] 依赖检查功能正常
- [ ] 视频录制功能正常（摄像头模式）
- [ ] 视频录制功能正常（回调模式）
- [ ] 音频录制功能正常
- [ ] 启动/停止功能正常
- [ ] 资源清理正常
- [ ] 错误处理正常
- [ ] 音视频同步正常（< 100ms误差）

### InterviewState 测试

- [ ] 状态初始化正常
- [ ] 状态转换正常（TRACKING → INTERVIEW）
- [ ] 状态转换正常（INTERVIEW → TRACKING）
- [ ] 对话流程正常
- [ ] 跟随功能正常
- [ ] 录制生命周期正常
- [ ] 错误处理正常

### ROS2 集成测试

- [ ] 图像订阅正常
- [ ] 图像转换正常
- [ ] 命令触发正常
- [ ] 状态发布正常
- [ ] 完整流程正常

### 性能测试

- [ ] 帧率稳定性（30fps ± 1%）
- [ ] 内存使用正常（无泄漏）
- [ ] CPU使用正常（< 50%）
- [ ] 同步精度（< 100ms）

---

## 测试报告模板

### 测试结果记录

```markdown
## Interview 测试报告

### 测试环境
- 日期: YYYY-MM-DD
- 测试人员: XXX
- 系统版本: XXX
- ROS2版本: XXX

### 测试结果

#### 单元测试
- [ ] InterviewRecorder: PASS/FAIL
- [ ] InterviewState: PASS/FAIL

#### 集成测试
- [ ] InterviewRecorder集成: PASS/FAIL
- [ ] InterviewState集成: PASS/FAIL

#### ROS2集成测试
- [ ] ROS2 Topic集成: PASS/FAIL
- [ ] 完整流程: PASS/FAIL

#### 功能测试
- [ ] 手动测试: PASS/FAIL
- [ ] 自动化测试: PASS/FAIL

#### 性能测试
- [ ] 帧率稳定性: PASS/FAIL
- [ ] 同步精度: PASS/FAIL

### 问题记录
1. [问题描述]
   - 严重程度: High/Medium/Low
   - 状态: Open/Fixed

### 结论
[测试结论]
```

---

## 快速测试命令

```bash
# 运行所有Interview相关测试
pytest test/ -k interview -v

# 运行单元测试
pytest test/unit/test_interview*.py -v

# 运行集成测试（需要ROS2环境）
pytest test/integration/test_interview*.py -v

# 运行性能测试
pytest test/performance/test_interview*.py -v
```

---

## 注意事项

1. **环境要求**
   - 需要ROS2环境
   - 需要仿真器（用于完整测试）
   - 需要摄像头和麦克风（或mock）

2. **测试数据**
   - 确保有足够的磁盘空间
   - 确保临时目录可写

3. **依赖检查**
   - 测试前检查所有依赖是否安装
   - OpenCV, PyAudio, ffmpeg

4. **清理工作**
   - 测试后清理临时文件
   - 清理测试生成的录制文件

