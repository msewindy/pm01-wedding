# Action Pack 动作执行流程分析

本文档详细分析从 IDLE 状态的 action pack 设定到机器人关节执行的完整流程。

## 一、动作生成阶段（FSM 状态层）

### 1.1 IDLE 状态加载动作包

**位置**：`idle_state.py` 的 `_select_random_pack()`

**流程**：
- 从动作库选择 IDLE_SWAY 类型的动作包
- 调用基类方法 `load_action_pack(pack_name)`
- 保存 `_current_pack` 和 `_pack_start_time`

**关键代码**：
```python
# 选择动作包
pack_name = random.choice(available_packs)
if self.load_action_pack(pack_name):
    self._pack_start_time = self.get_current_time()
```

### 1.2 动作包执行（每个 FSM 周期）

**位置**：`idle_state.py` 的 `run()` → `execute_action_pack()`

**流程**：
1. 计算动作包经过时间
   ```python
   pack_elapsed = current_time - self._pack_start_time
   ```

2. 获取头部偏移角度（弧度）
   ```python
   head_offset = self._current_pack.get_head_offset(pack_elapsed)
   # 内部计算：head_amplitude * sin(2π * elapsed_time / period)
   ```

3. 转换为归一化坐标 (0-1)
   ```python
   max_offset = 0.5  # 最大偏移 0.5 弧度
   normalized_x = 0.5 + (head_offset / max_offset) * 0.5
   normalized_x = clamp(normalized_x, 0.0, 1.0)
   ```

4. 设置注视目标
   ```python
   self.set_look_at(normalized_x, 0.5)
   ```

### 1.3 设置注视目标

**位置**：`wedding_state.py` 的 `set_look_at()`

**流程**：
```python
self.fsm.data.motion.look_at_target[0] = x  # 归一化坐标 0-1
self.fsm.data.motion.look_at_target[1] = y  # 归一化坐标 0-1
```

**说明**：将归一化坐标存储到 FSM 共享数据中，供后续节点使用。

## 二、消息发布阶段（FSM 节点层）

### 2.1 FSM 主循环

**位置**：`wedding_fsm_node.py` 的 `_run_fsm()`

**执行频率**：50Hz（由 `fsm_rate` 参数控制）

**流程**：
- 调用 `fsm.run_once()` 执行状态逻辑
- 调用 `_publish_motion()` 发布动作指令
- 调用 `_publish_speech()` 发布语音指令
- 重置单帧感知数据

### 2.2 发布注视目标消息

**位置**：`wedding_fsm_node.py` 的 `_publish_motion()`

**流程**：
```python
msg = PointStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = "camera"
msg.point.x = float(motion.look_at_target[0])  # 归一化坐标 0-1
msg.point.y = float(motion.look_at_target[1])  # 归一化坐标 0-1
msg.point.z = 0.0
self.look_at_pub.publish(msg)  # 发布到 /wedding/motion/look_at
```

**说明**：
- 使用 `PointStamped` 消息类型
- 发布到 `/wedding/motion/look_at` 话题
- 坐标系统：归一化坐标（0=左侧，0.5=中心，1=右侧）

## 三、运动适配阶段（Motion Adapter 节点层）

### 3.1 订阅注视目标

**位置**：`motion_adapter_node.py` 的 `_on_look_at()`（回调函数）

**流程**：
```python
# 接收归一化坐标 (0-1)
x = msg.point.x  # 0=左侧, 0.5=中心, 1=右侧

# 转换为 -1 到 1 范围
x_offset = (x - 0.5) * 2.0  # -1 (左) 到 1 (右)

# 转换为目标偏移角度（弧度）
self.target_head_offset = x_offset * HEAD_YAW_LIMIT * 0.8
# HEAD_YAW_LIMIT = 0.6 rad ≈ 34°
# 实际最大偏移 = 0.6 * 0.8 = 0.48 rad ≈ 27.5°

self.target_waist_offset = x_offset * WAIST_YAW_LIMIT * 0.5
# WAIST_YAW_LIMIT = 0.4 rad ≈ 23°
# 实际最大偏移 = 0.4 * 0.5 = 0.2 rad ≈ 11.5°
```

**说明**：
- 将归一化坐标转换为角度偏移
- 头部和腰部使用不同的缩放系数
- 头部运动范围更大，腰部运动范围较小

### 3.2 控制循环平滑处理

**位置**：`motion_adapter_node.py` 的 `_control_loop()`

**执行频率**：500Hz（由 `control_rate` 参数控制）

**流程**：
```python
# 平滑过渡（避免突变）
self.current_head_offset += smooth_factor * (
    self.target_head_offset - self.current_head_offset
)
# smooth_factor = 0.02，每周期调整 2%
# 时间常数 ≈ 50 个周期 = 0.1 秒（@500Hz）

self.current_waist_offset += smooth_factor * (
    self.target_waist_offset - self.current_waist_offset
)
```

**说明**：
- 使用指数平滑算法避免突变
- 平滑系数为 0.02，响应时间约 0.1 秒
- 保证运动连续性和平滑性

### 3.3 生成关节命令

**位置**：`motion_adapter_node.py` 的 `_publish_joint_command()`

**流程**：
```python
# 1. 初始化所有关节（24 DOF）
msg.position = self.initial_positions.tolist()  # 基于初始位置

# 2. 设置 PD 参数
# 腿部 (J00-J11): KP=400, KD=5 (高刚度保持稳定)
# 手臂 (J13-J22): KP=100, KD=2 (低刚度)
# 腰部/头部 (J12, J23): KP=200, KD=3 (中等刚度)

# 3. 设置腰部位置（J12）
msg.position[WAIST_YAW_IDX] = (
    self.initial_positions[WAIST_YAW_IDX] + self.current_waist_offset
)

# 4. 设置头部位置（J23）
msg.position[HEAD_YAW_IDX] = (
    self.initial_positions[HEAD_YAW_IDX] + self.current_head_offset
)

# 5. 发布到 /hardware/joint_command
self.joint_cmd_pub.publish(msg)
```

**说明**：
- 必须控制所有 24 个关节
- 腿部使用高刚度保持稳定
- 头部和腰部基于初始位置加上偏移角度
- 发布到硬件接口控制机器人

## 四、数据流转换链路

完整的数据流转换过程如下：

```
Action Pack (动作包)
  ↓
[弧度] head_offset = head_amplitude * sin(2πt/T)
  ↓
[归一化] normalized_x = 0.5 + (head_offset / 0.5) * 0.5  (范围: 0-1)
  ↓
FSM Data: motion.look_at_target[0] = normalized_x
  ↓
ROS2 Message: PointStamped.point.x = normalized_x
  ↓
Motion Adapter: x_offset = (x - 0.5) * 2.0  (范围: -1 到 1)
  ↓
[弧度] target_head_offset = x_offset * 0.6 * 0.8  (最大 ±0.48 rad)
  ↓
[平滑] current_head_offset += 0.02 * (target - current)
  ↓
[关节位置] joint_position[23] = initial_position[23] + current_head_offset
  ↓
JointCommand Message → 机器人硬件
```

### 转换说明

1. **动作包层**：生成正弦波偏移角度（弧度）
2. **FSM 状态层**：转换为归一化坐标（0-1）
3. **FSM 节点层**：封装为 ROS2 消息
4. **Motion Adapter 层**：转换为关节偏移角度（弧度）
5. **平滑处理**：指数平滑避免突变
6. **关节命令**：生成最终关节位置
7. **硬件接口**：发送到机器人执行

## 五、关键参数映射关系

| 阶段 | 参数 | 范围/单位 | 说明 |
|------|------|-----------|------|
| Action Pack | `head_amplitude` | 弧度 (rad) | 动作包定义的摆动幅度 |
| Action Pack | `period` | 秒 (s) | 摆动周期 |
| FSM State | `normalized_x` | 0.0 - 1.0 | 归一化坐标（0=左，0.5=中，1=右） |
| Motion Adapter | `x_offset` | -1.0 - 1.0 | 偏移系数 |
| Motion Adapter | `target_head_offset` | ±0.48 rad | 目标头部偏移（最大约 ±27.5°） |
| Motion Adapter | `HEAD_YAW_LIMIT` | 0.6 rad | 头部 Yaw 限位（约 34°） |
| Joint Command | `position[23]` | 弧度 (rad) | 最终关节位置 |

### 参数计算示例

假设动作包参数：
- `head_amplitude = 0.3 rad`（约 17.2°）
- `period = 2.5 s`

在 `t = 0.625 s` 时（1/4 周期）：
1. `head_offset = 0.3 * sin(π/2) = 0.3 rad`
2. `normalized_x = 0.5 + (0.3 / 0.5) * 0.5 = 0.8`
3. `x_offset = (0.8 - 0.5) * 2.0 = 0.6`
4. `target_head_offset = 0.6 * 0.6 * 0.8 = 0.288 rad`（约 16.5°）
5. 经过平滑后，`current_head_offset ≈ 0.288 rad`
6. `joint_position[23] = initial_position[23] + 0.288 rad`

## 六、执行频率

不同阶段的执行频率如下：

| 阶段 | 节点/模块 | 频率 | 说明 |
|------|-----------|------|------|
| FSM 主循环 | `wedding_fsm_node.py` | 50Hz | 由 `fsm_rate` 参数控制 |
| 动作包计算 | `idle_state.py` | 50Hz | 每个 FSM 周期执行一次 |
| 注视目标发布 | `wedding_fsm_node.py` | 50Hz | 每个 FSM 周期发布一次 |
| Motion Adapter 控制 | `motion_adapter_node.py` | 500Hz | 由 `control_rate` 参数控制 |
| 关节命令发布 | `motion_adapter_node.py` | 500Hz | 每个控制周期发布一次 |

### 频率关系说明

- **FSM 层（50Hz）**：状态逻辑和动作包计算
- **Motion Adapter 层（500Hz）**：高频率控制循环，保证运动平滑
- **频率比**：Motion Adapter 是 FSM 的 10 倍，确保即使 FSM 更新较慢，控制也能保持平滑

## 七、平滑机制

系统在两个层面实现平滑处理，确保运动连续性和自然性：

### 7.1 Action Pack 层平滑

**机制**：正弦波自然平滑

**特点**：
- 使用正弦函数生成平滑的周期性运动
- 速度在极值点为零，自然过渡
- 无需额外平滑处理

**公式**：
```
head_offset(t) = head_amplitude * sin(2π * t / period)
```

### 7.2 Motion Adapter 层平滑

**机制**：指数平滑（Exponential Smoothing）

**特点**：
- 平滑系数：`smooth_factor = 0.02`
- 每周期调整 2%，避免突变
- 时间常数：约 50 个周期 = 0.1 秒（@500Hz）

**公式**：
```
current_offset += smooth_factor * (target_offset - current_offset)
```

**响应特性**：
- **快速响应**：目标变化后，约 0.1 秒达到 63% 的目标值
- **平滑过渡**：避免阶跃变化，减少机械冲击
- **稳定性**：高频控制循环保证平滑性

### 7.3 平滑效果对比

| 场景 | 无平滑 | 有平滑 |
|------|--------|--------|
| 目标突变 | 关节位置突变，可能产生冲击 | 平滑过渡，无冲击 |
| 运动连续性 | 可能出现不连续 | 连续平滑 |
| 机械磨损 | 较大 | 较小 |
| 视觉观感 | 可能显得生硬 | 自然流畅 |

## 八、总结

### 8.1 完整执行链路

从 Action Pack 到机器人关节的完整执行链路包括：

1. **动作包生成**：正弦波偏移角度（弧度）
2. **坐标转换**：转换为归一化坐标（0-1）
3. **消息发布**：FSM 节点发布 ROS2 消息
4. **角度转换**：Motion Adapter 转换为关节偏移（弧度）
5. **平滑处理**：指数平滑保证连续性
6. **关节命令**：生成最终关节位置
7. **硬件执行**：发布到硬件接口控制机器人

### 8.2 关键设计要点

#### 坐标系统统一
- **FSM 层**：统一使用归一化坐标（0-1）传递
- **Motion Adapter 层**：转换为角度（弧度）控制关节
- **优势**：解耦坐标系统和硬件实现

#### 角度单位统一
- **动作包层**：使用弧度（rad）定义动作幅度
- **关节层**：使用弧度（rad）控制关节位置
- **优势**：避免单位转换错误

#### 平滑处理
- **两层平滑**：Action Pack 正弦波 + Motion Adapter 指数平滑
- **优势**：保证运动连续性和自然性

#### 全关节控制
- **必须控制所有 24 个关节**
- **腿部高刚度**：保持机器人稳定
- **头部/腰部中等刚度**：允许跟随运动
- **手臂低刚度**：减少能耗

### 8.3 系统优势

1. **模块化设计**：各层职责清晰，易于维护
2. **参数可配置**：频率、平滑系数等可调
3. **实时性好**：高频率控制循环保证响应速度
4. **平滑自然**：双重平滑机制保证运动质量
5. **稳定性高**：腿部高刚度保持机器人稳定

### 8.4 扩展性

该架构支持以下扩展：

- **新增动作类型**：在 Action Pack 中定义新的动作模式
- **调整运动范围**：修改 `HEAD_YAW_LIMIT` 和 `WAIST_YAW_LIMIT`
- **优化平滑参数**：调整 `smooth_factor` 改变响应速度
- **添加新关节**：在 Motion Adapter 中扩展关节控制逻辑

---

**文档版本**：v1.0  
**最后更新**：2024年

