# 婚礼机器人语音播放方案

## 1. 概述

本文档描述婚礼互动机器人的语音播放技术方案。

### 1.1 核心约束

- **Jetson 真机离线**：无法直接使用在线 TTS 服务
- **音质要求高**：需要自然、流畅的语音，避免机械感
- **声音一致性**：机器人必须使用统一的声音

### 1.2 解决方案

**分布式架构：Host（联网） + Jetson（离线）**

- **固定语音**（问候、倒计时）：使用 Edge-TTS 预先生成 MP3，部署到 Jetson 离线播放
- **动态对话**（大模型输出）：Host 上用 Edge-TTS 实时生成，通过网络传输到 Jetson 播放
- **结果**：机器人始终使用同一种声音（微软小晓 XiaoxiaoNeural）

```
┌─────────────────────────────────────────────────────────────────────┐
│                   分布式语音方案（Edge-TTS）                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   Host（联网）                         Jetson（离线）               │
│   ┌───────────────────┐                ┌───────────────────┐       │
│   │   Edge-TTS        │                │   AudioPlayer     │       │
│   │   (在线服务)       │                │   (ffplay)        │       │
│   └─────────┬─────────┘                └─────────┬─────────┘       │
│             │                                    │                  │
│             │                                    │                  │
│   ┌─────────┴─────────────────┐      ┌──────────┴──────────┐       │
│   │                           │      │                      │       │
│   │  1. 预生成固定语音        │      │  audio_resources/   │       │
│   │     (开发阶段)            │ ───▶ │  *.mp3 文件         │       │
│   │                           │      │  (26条预录音频)      │       │
│   │  2. 实时生成动态语音      │      │                      │       │
│   │     (大模型对话输出)      │ ───▶ │  网络接收 MP3 流     │       │
│   │                           │      │                      │       │
│   └───────────────────────────┘      └──────────────────────┘       │
│                                                                     │
│   ════════════════════════════════════════════════════════════     │
│             同一种声音（微软小晓 XiaoxiaoNeural）                    │
│   ════════════════════════════════════════════════════════════     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.3 为什么选择 Edge-TTS？

| 特性 | Edge-TTS | Piper TTS |
|-----|----------|-----------|
| **音质** | ★★★★★ 极自然 | ★★★☆☆ 较机械 |
| **中文支持** | 微软神经网络语音 | 开源模型 |
| **延迟** | ~500ms（需网络） | ~150ms |
| **离线** | ❌ 需要网络 | ✅ 完全离线 |

**结论**：Edge-TTS 音质远优于 Piper，通过 Host 联网的方式解决 Jetson 离线问题。

## 2. 方案架构

### 2.1 两种语音类型

| 类型 | 数量 | 生成时机 | 生成位置 | 播放位置 |
|-----|------|---------|---------|---------|
| **固定语音** | 26条 | 开发阶段预生成 | Host | Jetson |
| **动态语音** | 无限 | 运行时实时生成 | Host | Jetson |

### 2.2 固定语音流程

```
开发阶段（一次性）：
Host ──Edge-TTS──▶ audio_resources/*.mp3 ──复制──▶ Jetson

运行时：
Jetson ──播放──▶ audio_resources/idle_greeting.mp3
```

### 2.3 动态语音流程（大模型对话）

```
运行时：
LLM输出 ──▶ Host ──Edge-TTS──▶ MP3流 ──网络──▶ Jetson ──播放
```

## 3. Edge-TTS 使用指南

### 3.1 安装

```bash
pip install edge-tts
```

### 3.2 可用声音

| 声音 ID | 性别 | 风格 | 推荐 |
|--------|------|------|------|
| zh-CN-XiaoxiaoNeural | 女 | 自然、亲切 | ⭐⭐⭐ |
| zh-CN-XiaoyiNeural | 女 | 活泼、年轻 | |
| zh-CN-YunjianNeural | 男 | 新闻播报 | |
| zh-CN-YunxiNeural | 男 | 活泼 | |

### 3.3 命令行使用

```bash
# 生成单个音频
edge-tts --voice zh-CN-XiaoxiaoNeural --text "你好，欢迎来到婚礼现场！" --write-media hello.mp3

# 播放
ffplay -nodisp -autoexit hello.mp3
```

### 3.4 Python API

```python
import edge_tts
import asyncio

async def generate_speech(text: str, output_file: str):
    communicate = edge_tts.Communicate(text, "zh-CN-XiaoxiaoNeural")
    await communicate.save(output_file)

asyncio.run(generate_speech("你好！", "output.mp3"))
```

## 4. 预录音频生成

### 4.1 批量生成脚本

```bash
cd /home/lingjing/project/engine_ai/wedding_jeston

# 列出所有待生成的语音
python scripts/generate_audio.py --list

# 使用 Edge-TTS 生成所有音频【推荐】
python scripts/generate_audio.py --engine edge --voice zh-CN-XiaoxiaoNeural

# 生成特定语音
python scripts/generate_audio.py --engine edge --only "idle_greeting,photo_countdown_3"
```

### 4.2 部署到 Jetson

```bash
# 在开发电脑上打包
cd /home/lingjing/project/engine_ai/wedding_jeston
tar czf audio_resources.tar.gz audio_resources/

# 传输到 Jetson
scp audio_resources.tar.gz jetson@192.168.x.x:/home/jetson/wedding_robot/

# 在 Jetson 上解压
cd /home/jetson/wedding_robot
tar xzf audio_resources.tar.gz

# 验证（使用 ffplay 播放 MP3）
ffplay -nodisp -autoexit audio_resources/idle_greeting.mp3
```

## 5. 预定义语音库（26条）

### 5.1 分类

| 类别 | 数量 | 用途 |
|-----|------|------|
| IDLE 待机 | 4 | 空闲状态随机播放 |
| 问候 | 4 | 检测到人时主动问候 |
| 合影 | 7 | 合影流程引导 |
| 送别 | 3 | 离开时送别 |
| 采访 | 4 | 采访引导 |
| 系统 | 3 | 系统提示 |

### 5.2 完整列表

```yaml
# IDLE 待机
idle_greeting: "你好呀，欢迎来到婚礼现场！"
idle_welcome: "欢迎欢迎，今天是个好日子！"
idle_random_1: "有什么可以帮你的吗？"
idle_random_2: "需要合影吗？摆个 Pose 就可以啦！"

# 问候
greeting_hello: "你好！很高兴见到你！"
greeting_nice: "哇，你今天真好看！"
greeting_bride: "恭喜恭喜！新娘子真美！"
greeting_groom: "恭喜恭喜！新郎官好帅！"

# 合影
photo_ready: "准备好了吗？"
photo_countdown_3: "三！"
photo_countdown_2: "二！"
photo_countdown_1: "一！"
photo_cheese: "茄子！"
photo_done: "拍好啦！照片很棒哦！"
photo_again: "再来一张吗？"

# 送别
farewell_bye: "再见！祝你们幸福美满！"
farewell_happy: "百年好合！永结同心！"
farewell_thanks: "谢谢光临！玩得开心！"

# 采访
interview_start: "让我来采访一下！"
interview_question_1: "请问你和新人是什么关系呀？"
interview_question_2: "有什么祝福想对新人说的吗？"
interview_question_3: "来分享一个你和新人的小故事吧！"
interview_thanks: "谢谢你的祝福！"

# 系统
system_error: "抱歉，出了点小问题。"
system_too_close: "请稍微后退一点点哦。"
system_starting: "系统启动中，请稍等。"
```

## 6. 软件架构

### 6.1 模块结构

```
wedding_interaction/
├── audio/
│   ├── __init__.py
│   ├── audio_player.py      # 底层音频播放（支持 MP3）
│   └── speech_manager.py    # 语音管理器（队列、播放）
├── nodes/
│   └── speech_player_node.py  # ROS2 节点
└── ...

audio_resources/               # 预录音频文件目录
├── idle_greeting.mp3         # Edge-TTS 生成
├── photo_countdown_3.mp3
└── ... (共 26 个 MP3 文件)

scripts/
└── generate_audio.py          # 音频生成脚本
```

### 6.2 播放器选择

| 文件格式 | 播放器 | 命令 |
|---------|--------|------|
| MP3 | ffplay | `ffplay -nodisp -autoexit -loglevel quiet file.mp3` |
| WAV | aplay | `aplay -q file.wav` |

**注意**：aplay 不支持 MP3，必须用 ffplay 或 mpv！

### 6.3 ROS2 接口

**订阅**：
| Topic | 类型 | 说明 |
|-------|------|------|
| `/wedding/audio/play` | `std_msgs/String` | 播放请求（ID 或文本） |
| `/wedding/audio/stop` | `std_msgs/Bool` | 停止播放 |

**发布**：
| Topic | 类型 | 说明 |
|-------|------|------|
| `/wedding/audio/playing` | `std_msgs/Bool` | 是否正在播放 |
| `/wedding/audio/current` | `std_msgs/String` | 当前播放的语音 ID |

### 6.4 播放请求格式

```bash
# 播放预录语音（通过 ID）
ros2 topic pub --once /wedding/audio/play std_msgs/String "data: 'idle_greeting'"

# 打断式播放
ros2 topic pub --once /wedding/audio/play std_msgs/String "data: 'interrupt:photo_countdown_3'"
```

## 7. 配置说明

### 7.1 Launch 文件参数

```python
# idle_debug.launch.py
speech_player_node = Node(
    package='wedding_interaction',
    executable='speech_player',
    name='speech_player_node',
    parameters=[
        {'audio_dir': '/path/to/audio_resources'},  # 预录音频目录
        {'tts_enabled': True},
        {'tts_engine': 'mock'},  # 仿真模式
    ],
)
```

### 7.2 环境配置

**仿真环境（开发调试）**：
```yaml
tts_enabled: true
tts_engine: mock  # 模拟模式，打印日志
audio_dir: /home/lingjing/project/engine_ai/wedding_jeston/audio_resources
```

**真机环境（Jetson）**：
```yaml
tts_enabled: true
tts_engine: mock  # 固定语音不需要实时 TTS
audio_dir: /home/jetson/wedding_robot/audio_resources
```

## 8. 动态语音（大模型对话）

### 8.1 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│   Jetson                              Host（联网）              │
│   ┌─────────────┐                    ┌─────────────┐           │
│   │   LLM 推理  │ ──文本输出──────▶  │  Edge-TTS   │           │
│   │   (本地)    │                    │  (在线)     │           │
│   └─────────────┘                    └──────┬──────┘           │
│                                             │                   │
│                                      MP3 音频流                 │
│                                             │                   │
│   ┌─────────────┐                          │                   │
│   │ AudioPlayer │ ◀─────────网络传输───────┘                   │
│   │  (ffplay)   │                                              │
│   └─────────────┘                                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 8.2 实现方式（后续开发）

1. **Jetson 端**：LLM 生成文本 → 发送到 Host
2. **Host 端**：接收文本 → Edge-TTS 生成 MP3 → 发送回 Jetson
3. **Jetson 端**：接收 MP3 → ffplay 播放

## 9. 部署检查清单

### 9.1 开发电脑（Host）

- [x] 安装 edge-tts: `pip install edge-tts`
- [x] 运行 `generate_audio.py --engine edge` 生成所有音频
- [x] 验证音频文件: `ffplay -nodisp -autoexit audio_resources/idle_greeting.mp3`
- [ ] 打包传输到 Jetson

### 9.2 Jetson 真机

- [ ] 安装 ffplay: `sudo apt install ffmpeg`
- [ ] 复制 audio_resources/ 目录
- [ ] 测试播放: `ffplay -nodisp -autoexit audio_resources/idle_greeting.mp3`
- [ ] 配置 launch 文件中的 audio_dir 路径

## 10. 故障排除

### 10.1 无声音输出

```bash
# 检查音频设备
aplay -l

# 测试音频输出
speaker-test -t wav -c 2

# 检查音量
alsamixer
```

### 10.2 MP3 播放问题

```bash
# 确认 ffplay 已安装
which ffplay

# 手动测试
ffplay -nodisp -autoexit -loglevel quiet audio_resources/idle_greeting.mp3
```

### 10.3 ROS2 通信问题

```bash
# 检查节点
ros2 node list | grep speech

# 检查 topic
ros2 topic list | grep audio

# 手动测试
ros2 topic pub --once /wedding/audio/play std_msgs/String "data: 'idle_greeting'"
```

---

## 附录：快速参考

### A. 常用命令

```bash
# 安装 edge-tts
pip install edge-tts

# 生成预录音频
python scripts/generate_audio.py --engine edge --voice zh-CN-XiaoxiaoNeural

# 播放测试
ffplay -nodisp -autoexit audio_resources/idle_greeting.mp3

# ROS2 播放
ros2 topic pub --once /wedding/audio/play std_msgs/String "data: 'idle_greeting'"
```

### B. 文件位置

| 文件 | 路径 |
|------|------|
| 生成脚本 | `scripts/generate_audio.py` |
| 语音管理器 | `wedding_interaction/audio/speech_manager.py` |
| ROS2 节点 | `wedding_interaction/nodes/speech_player_node.py` |
| 音频目录 | `audio_resources/` (26 个 MP3 文件) |

### C. 音频格式

| 格式 | 来源 | 播放器 | 音质 |
|-----|------|--------|------|
| MP3 | Edge-TTS | ffplay | ★★★★★ |
| WAV | Piper（已弃用） | aplay | ★★★☆☆ |
