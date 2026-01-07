# 测试脚本重组总结

## 重组完成时间
2025-01-03

## 重组内容

### 移动的测试脚本

从 `scripts/` 移动到 `test/` 的测试脚本：

#### 集成测试 (test/integration/)
- `test_idle_search_integration.py` - IDLE→SEARCH集成测试
- `test_fsm_standalone.py` - FSM独立集成测试

#### 感知测试 (test/perception/)
- `test_perception.py` - 感知模块主测试
- `test_image_face_detection.py` - 图像人脸检测测试
- `test_sim_face_detection.py` - 仿真环境人脸检测测试
- `test_min_face_size.py` - 最小人脸尺寸测试

#### 状态测试 (test/states/)
- `test_search_state.py` - SEARCH状态独立测试

#### 调试工具 (test/debug/)
- `debug_perception.py` - 感知模块调试工具
- `debug_false_detection.py` - 误检测调试工具

#### 测试工具 (test/utils/)
- `mock_perception.py` - 感知数据Mock工具

### 保留在 scripts/ 的工具脚本

以下脚本保留在 `scripts/` 目录，因为它们不是测试脚本：

- `view_visualization.py` - 可视化工具
- `photo_panel_keyboard_control.py` - 控制工具
- `download_mediapipe_models.py` - 模型下载工具
- `generate_audio.py` - 音频生成工具
- `setup_sim_env.sh` - 环境设置脚本
- `run_tests.sh` - 测试运行脚本（已更新）

### 单元测试 (test/unit/)

原有的单元测试已移动到 `test/unit/`：

- `test_enums.py` - 枚举类型测试
- `test_fsm_data.py` - FSM数据模型测试
- `test_fsm.py` - FSM核心逻辑测试
- `test_states.py` - 状态基类测试
- `test_idle_search_tracking_logic.py` - IDLE/SEARCH/TRACKING逻辑测试

## 更新的内容

### 1. 路径引用更新

所有移动的脚本中的路径引用已更新：
- Usage说明中的路径已更新
- `sys.path.insert` 已改为使用相对路径

### 2. 创建的文件

- 各子目录的 `__init__.py` 文件
- `test/README.md` - 测试目录说明文档
- `test/REORGANIZATION_SUMMARY.md` - 本文件

### 3. 更新的文件

- `scripts/run_tests.sh` - 支持按类型运行测试

## 目录结构

```
test/
├── unit/              # 单元测试
├── integration/        # 集成测试
├── perception/        # 感知模块测试
├── states/            # 状态测试
├── debug/             # 调试工具
├── utils/             # 测试工具
├── README.md          # 测试目录说明
└── REORGANIZATION_SUMMARY.md  # 重组总结
```

## 使用说明

### 运行所有测试

```bash
./scripts/run_tests.sh
```

### 运行特定类型测试

```bash
./scripts/run_tests.sh unit          # 单元测试
./scripts/run_tests.sh integration   # 集成测试
./scripts/run_tests.sh perception    # 感知测试
./scripts/run_tests.sh states        # 状态测试
```

### 运行单个测试

```bash
# 使用pytest
pytest test/unit/test_idle_search_tracking_logic.py -v

# 直接运行脚本
python3 test/integration/test_idle_search_integration.py
```

## 注意事项

1. 集成测试需要先启动ROS2节点
2. 感知测试可能需要摄像头或图像文件
3. 所有脚本的路径引用已更新，应该可以正常工作
4. 如果遇到导入错误，检查 `sys.path` 设置是否正确

