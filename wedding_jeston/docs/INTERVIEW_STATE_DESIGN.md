# 采访模式设计方案

## 需求分析

根据用户需求，采访模式需要实现：
1. **跟随目标**：在一定范围内跟随目标（与 TRACKING 相同）
2. **特定动作**：右手拿着麦克风抬起
3. **录制功能**：相机视频流 + 麦克风音频流写入媒体文件
4. **对话逻辑**：简单的一轮固定逻辑对话
5. **状态转换**：采访结束后回到 TRACKING 模式

## 方案对比

### 方案 A：独立状态（INTERVIEW）

#### 架构设计
```
TRACKING → [触发采访] → INTERVIEW → [采访完成] → TRACKING
```

#### 优点
1. **职责清晰**：每个状态职责单一，符合单一职责原则
2. **生命周期管理简单**：
   - `on_enter()`: 开始录制、设置麦克风动作
   - `run()`: 跟随目标、执行对话逻辑
   - `on_exit()`: 停止录制、保存文件、恢复动作
3. **状态转换明确**：状态转换逻辑清晰，便于调试
4. **符合现有架构**：与 `PHOTO_POSING` 状态设计模式一致
5. **便于扩展**：未来可以独立扩展采访功能（多轮对话、手势控制等）
6. **状态可观测**：外部可以通过 `/wedding/fsm/state` 明确知道当前在采访状态

#### 缺点
1. **代码复用**：需要复用 TRACKING 的跟随逻辑（可通过基类方法解决）
2. **状态转换开销**：多一次状态转换（开销可忽略）

#### 实现要点
- 继承 `WeddingState`，复用基类的 `follow_target_face()` 方法
- 在 `on_enter()` 中启动录制服务
- 在 `run()` 中执行跟随 + 对话逻辑
- 在 `on_exit()` 中停止录制并保存文件

---

### 方案 B：TRACKING 状态下的功能

#### 架构设计
```
TRACKING (内部有 interview_mode 标志)
  - 正常模式：跟随 + 对话
  - 采访模式：跟随 + 麦克风动作 + 录制 + 对话
```

#### 优点
1. **代码复用**：跟随逻辑直接复用，无需额外实现
2. **实现简单**：不需要新状态类，只需在 TRACKING 中添加标志和逻辑
3. **状态转换少**：无需状态转换，只需切换内部模式

#### 缺点
1. **职责过重**：TRACKING 状态需要管理两种模式，违反单一职责原则
2. **生命周期管理复杂**：
   - 需要在 `run()` 中判断是否在采访模式
   - 录制开始/结束逻辑分散在多个地方
   - 错误处理复杂（如果录制失败，需要回退状态）
3. **状态不明确**：外部无法明确区分"跟随"和"采访"状态
4. **不利于扩展**：如果未来采访功能变复杂，TRACKING 状态会变得臃肿
5. **测试困难**：需要测试两种模式的组合，测试用例复杂

#### 实现要点
- 在 TRACKING 中添加 `_interview_mode: bool` 标志
- 在 `check_transition()` 中处理采访触发
- 在 `run()` 中根据标志执行不同逻辑
- 需要管理录制生命周期（何时开始、何时结束）

---

## 推荐方案：独立状态（方案 A）

### 推荐理由

1. **符合设计原则**：
   - 单一职责：每个状态只负责一种行为
   - 开闭原则：新增采访功能不需要修改 TRACKING 状态
   - 可维护性：代码结构清晰，易于理解和维护

2. **与现有架构一致**：
   - `PHOTO_POSING` 也是独立状态，设计模式一致
   - 状态转换逻辑统一（`check_transition()` → `transition()` → `on_exit()` → `on_enter()`）

3. **生命周期管理清晰**：
   ```
   on_enter()  → 开始录制、设置麦克风动作
   run()       → 跟随目标、执行对话
   on_exit()   → 停止录制、保存文件、恢复动作
   ```

4. **便于测试和调试**：
   - 可以独立测试采访状态
   - 状态转换日志清晰
   - 问题定位容易

5. **代码复用简单**：
   - 基类 `WeddingState` 已提供 `follow_target_face()` 方法
   - 只需调用基类方法即可实现跟随

---

## 详细设计方案

### 1. 状态类结构

```python
class InterviewState(WeddingState):
    """
    采访状态
    
    行为：
    - 跟随目标（复用基类方法）
    - 右手拿麦克风动作
    - 录制视频+音频
    - 执行固定对话逻辑
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.INTERVIEW)
        
        # 跟随参数（与 TRACKING 相同）
        self.follow_smooth = 0.25
        
        # 采访流程阶段
        self.phase = "start"  # start, greeting, question, listening, ending, done
        
        # 录制管理
        self._recorder = None  # 录制器实例
        self._recording_path = None
        
        # 对话逻辑
        self._question_index = 0
        self._questions = [
            "您好，我是今天的特约记者，能送给新人一句祝福吗？",
            "简单说一句祝福吧！"
        ]
        
    def on_enter(self) -> None:
        """进入采访状态"""
        # 1. 启动录制
        self._start_recording()
        
        # 2. 设置麦克风动作
        self.set_pose("interview_mic_hold")
        
        # 3. 初始化对话状态
        self.phase = "start"
        self._question_index = 0
        
    def run(self) -> None:
        """执行采访逻辑"""
        # 1. 跟随目标（复用基类方法）
        target_face = self._find_tracked_face(...)
        self.follow_target_face(target_face, self.follow_smooth)
        
        # 2. 执行对话流程
        self._execute_interview_flow()
        
    def check_transition(self) -> WeddingStateName:
        """检查状态转换"""
        # 采访完成 -> 回到 TRACKING
        if self.phase == "done":
            return WeddingStateName.TRACKING
        
        return self.state_name
    
    def on_exit(self) -> None:
        """退出采访状态"""
        # 1. 停止录制
        self._stop_recording()
        
        # 2. 恢复动作
        self.set_pose("neutral")
```

### 2. 跟随逻辑复用

**方案**：使用基类的 `follow_target_face()` 方法

```python
# 在 InterviewState.run() 中
def run(self) -> None:
    # 查找跟踪的目标（使用基类方法）
    target_face = self._find_tracked_face(perception.faces)
    
    # 跟随目标（使用基类方法，与 TRACKING 相同）
    has_target = self.follow_target_face(
        target_face, 
        self.follow_smooth, 
        "[INTERVIEW跟随]",
        update_tracked_position=True
    )
    
    # 更新目标丢失时间
    self.update_target_lost_time(has_target)
```

### 3. 录制功能设计

#### 录制器接口（建议独立模块）

```python
class InterviewRecorder:
    """采访录制器"""
    
    def __init__(self, save_path: str):
        self.save_path = save_path
        self._video_writer = None
        self._audio_writer = None
        self._is_recording = False
    
    def start(self, video_source, audio_source) -> bool:
        """开始录制"""
        # 初始化视频/音频写入器
        # 返回是否成功
    
    def write_frame(self, frame, audio_chunk):
        """写入一帧数据"""
        # 在 run() 中每帧调用
    
    def stop(self) -> str:
        """停止录制并保存文件"""
        # 返回保存的文件路径
```

#### 在状态中使用

```python
def on_enter(self) -> None:
    # 创建录制器
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    self._recording_path = f"/home/user/wedding_blessings/blessing_{timestamp}.mp4"
    self._recorder = InterviewRecorder(self._recording_path)
    
    # 启动录制
    video_source = self.fsm.data.perception.camera_stream  # 从感知数据获取
    audio_source = self.fsm.data.audio.microphone_stream  # 从音频数据获取
    self._recorder.start(video_source, audio_source)

def run(self) -> None:
    # ... 跟随逻辑 ...
    
    # 写入录制数据
    if self._recorder and self._recorder.is_recording:
        frame = self.fsm.data.perception.current_frame
        audio = self.fsm.data.audio.current_audio_chunk
        self._recorder.write_frame(frame, audio)

def on_exit(self) -> None:
    # 停止录制
    if self._recorder:
        saved_path = self._recorder.stop()
        self.log(f"Interview recording saved: {saved_path}")
        self._recorder = None
```

### 4. 对话流程设计

```python
def _execute_interview_flow(self) -> None:
    """执行采访流程"""
    elapsed = self.get_elapsed_time()
    
    if self.phase == "start":
        # 播放开场白
        self.set_speech("interview_start")
        self.phase = "greeting"
        self.phase_start_time = self.fsm.get_current_time()
        
    elif self.phase == "greeting":
        # 等待开场白播放完成（假设 3 秒）
        if elapsed > 3.0:
            self.phase = "question"
            self._ask_question()
            
    elif self.phase == "question":
        # 等待问题播放完成（假设 2 秒）
        if elapsed - self.phase_start_time > 2.0:
            self.phase = "listening"
            self.listening_start_time = self.fsm.get_current_time()
            
    elif self.phase == "listening":
        # 监听用户回答
        # 检测语音停顿或超时
        silence_duration = self._get_silence_duration()
        listening_duration = elapsed - self.listening_start_time
        
        if silence_duration > 1.5 or listening_duration > 10.0:
            # 用户回答完成或超时
            self.phase = "ending"
            self.set_speech("interview_end")
            self.phase_start_time = self.fsm.get_current_time()
            
    elif self.phase == "ending":
        # 等待结束语播放完成
        if elapsed - self.phase_start_time > 2.0:
            self.phase = "done"
            
    elif self.phase == "done":
        # 等待一小段时间后退出
        if elapsed - self.phase_start_time > 1.0:
            pass  # check_transition 会处理
```

### 5. 状态转换触发

#### 从 TRACKING 触发

```python
# 在 TrackingState.check_transition() 中
def check_transition(self) -> WeddingStateName:
    base_next = super().check_transition()
    if base_next != self.state_name:
        return base_next
    
    # 检查采访触发
    if self.fsm.data.pending_command == WeddingEvent.CMD_START_INTERVIEW:
        return WeddingStateName.INTERVIEW
    
    # 或者通过语音命令
    if self.fsm.data.audio.voice_command == "interview":
        return WeddingStateName.INTERVIEW
    
    return self.state_name
```

#### 从 INTERVIEW 返回

```python
# 在 InterviewState.check_transition() 中
def check_transition(self) -> WeddingStateName:
    base_next = super().check_transition()
    if base_next != self.state_name:
        return base_next
    
    # 采访完成
    if self.phase == "done":
        return WeddingStateName.TRACKING
    
    # 外部命令中断
    if self.fsm.data.pending_command == WeddingEvent.CMD_STOP:
        return WeddingStateName.TRACKING
    
    return self.state_name
```

---

## 实现步骤

### Step 1: 创建 InterviewState 类
- [ ] 创建 `interview_state.py`
- [ ] 实现基本结构（`on_enter`, `run`, `check_transition`, `on_exit`）
- [ ] 复用基类的跟随方法

### Step 2: 实现录制功能
- [ ] 创建 `InterviewRecorder` 类（独立模块）
- [ ] 实现视频+音频录制逻辑
- [ ] 在状态中集成录制器

### Step 3: 实现对话流程
- [ ] 实现固定对话逻辑
- [ ] 集成语音检测（VAD）
- [ ] 添加超时处理

### Step 4: 添加麦克风动作
- [ ] 定义 `interview_mic_hold` Pose
- [ ] 在 `on_enter()` 中设置
- [ ] 在 `on_exit()` 中恢复

### Step 5: 状态转换集成
- [ ] 在 `TrackingState` 中添加触发逻辑
- [ ] 在 `WeddingFSMNode` 中添加命令处理
- [ ] 测试状态转换

### Step 6: 测试与优化
- [ ] 单元测试
- [ ] 集成测试
- [ ] 真机测试

---

## 配置参数

```yaml
interview:
  # 跟随参数
  follow_smooth_factor: 0.25      # 跟随平滑系数（与 TRACKING 相同）
  
  # 录制参数
  save_path: "/home/user/wedding_blessings/"
  max_duration: 60.0               # 最大录制时长（秒）
  min_duration: 5.0                # 最小录制时长（秒）
  video_fps: 30                    # 视频帧率
  audio_sample_rate: 44100         # 音频采样率
  
  # 对话参数
  greeting_duration: 3.0           # 开场白时长（秒）
  question_duration: 2.0           # 问题播放时长（秒）
  listening_timeout: 10.0          # 监听超时（秒）
  silence_threshold: 1.5           # 静音检测阈值（秒）
  ending_duration: 2.0            # 结束语时长（秒）
```

---

## 总结

**推荐使用独立状态（方案 A）**，理由：
1. ✅ 符合设计原则，代码结构清晰
2. ✅ 与现有架构一致（类似 PHOTO_POSING）
3. ✅ 生命周期管理简单（录制开始/结束逻辑清晰）
4. ✅ 便于测试和扩展
5. ✅ 代码复用简单（使用基类方法）

**实现复杂度**：中等（需要实现录制功能，但状态逻辑相对简单）

**维护成本**：低（职责清晰，易于维护）

