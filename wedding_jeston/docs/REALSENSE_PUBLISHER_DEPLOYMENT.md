# RealSense Publisher Node 部署指南

## 节点功能
`realsense_publisher_node.py` 是一个 ROS2 节点，用于从 Intel RealSense 相机读取彩色图像并发布到 ROS2 topic。

## 需要复制的文件

### 方案一：最小化部署（仅测试节点）
如果只需要测试节点本身，复制以下文件：

```
wedding_jeston/wedding_interaction/nodes/realsense_publisher_node.py
```

### 方案二：完整包部署（推荐）
如果要作为 ROS2 包安装，复制以下目录结构：

```
wedding_jeston/
├── package.xml
├── setup.py
├── resource/
│   └── wedding_interaction
└── wedding_interaction/
    ├── __init__.py
    └── nodes/
        ├── __init__.py
        └── realsense_publisher_node.py
```

## 依赖安装

在目标机器上需要安装以下依赖：

### 1. ROS2 环境
确保已安装 ROS2（推荐 Humble 或更高版本）

### 2. Python 依赖
```bash
# ROS2 相关包（通常已随 ROS2 安装）
sudo apt install ros-<distro>-sensor-msgs ros-<distro>-cv-bridge

# Python 依赖（重要：使用 NumPy 1.x 版本，因为 cv_bridge 和 pyrealsense2 需要兼容）
pip3 install "numpy<2" pyrealsense2 opencv-python
```

**重要提示：** 必须使用 NumPy 1.x 版本（如 `numpy<2`），因为：
- `cv_bridge` 是用 NumPy 1.x 编译的，不支持 NumPy 2.x
- `pyrealsense2` 也可能存在类似的兼容性问题

### 3. RealSense SDK

**重要：** RealSense SDK 不在默认的 Ubuntu 软件源中，需要先添加 Intel 官方软件源。

#### 步骤 1：添加 Intel RealSense 软件源

```bash
# 1. 注册服务器的公钥
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# 2. 确保 apt 支持 HTTPS（如果未安装）
sudo apt-get install apt-transport-https

# 3. 添加 Intel RealSense 软件源
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

# 4. 更新软件包列表
sudo apt-get update
```

#### 步骤 2：检查软件源是否添加成功

```bash
# 检查软件源列表
cat /etc/apt/sources.list.d/librealsense.list

# 检查软件包是否可用
apt-cache search librealsense2
```

#### 步骤 3：安装 RealSense SDK 包

```bash
# 安装 RealSense SDK（内核模块、工具和开发文件）
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev

# 可选：安装调试符号（用于调试）
sudo apt-get install librealsense2-dbg
```

#### 步骤 4：验证安装

```bash
# 运行 RealSense Viewer 测试相机（需要系统级 SDK）
realsense-viewer
```

**注意：** 如果只使用 Python 包，可以用以下方式测试：
```bash
# 测试 pyrealsense2 是否可用（不检查版本）
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 导入成功')"
```

### 3.1 替代方案：仅使用 Python 包（推荐用于快速测试）

**如果系统级安装遇到问题，可以尝试仅使用 Python 包：**

对于 Python 节点，实际上可能只需要 `pyrealsense2` 包，它已经包含了必要的库：

```bash
# 直接安装 Python 包（重要：使用 NumPy 1.x）
pip3 install "numpy<2" pyrealsense2 opencv-python

# 测试是否可用（简单导入测试）
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 导入成功')"

# 或者更完整的测试（尝试创建 pipeline）
python3 -c "import pyrealsense2 as rs; p = rs.pipeline(); print('RealSense 初始化成功')"
```

**注意：** 
- 如果 `pyrealsense2` 安装成功且能正常导入，通常就可以运行节点了
- `pyrealsense2` 模块可能没有 `__version__` 属性，这不影响使用
- 系统级的 `librealsense2-*` 包主要用于系统工具（如 `realsense-viewer`）和开发
- 如果您的 Ubuntu 版本较新（如 Ubuntu 24.04），可能遇到包找不到的问题，此时使用 Python 包是更好的选择

## 运行方式

### 方式一：直接运行 Python 文件（最简单）

```bash
# 1. 设置 ROS2 环境
source /opt/ros/<distro>/setup.bash

# 2. 直接运行节点
python3 wedding_interaction/nodes/realsense_publisher_node.py
```

### 方式二：使用 ros2 run（需要安装包）

```bash
# 1. 设置 ROS2 环境
source /opt/ros/<distro>/setup.bash

# 2. 进入包目录，安装包
cd wedding_jeston
pip3 install -e .

# 3. 运行节点
ros2 run wedding_interaction realsense_publisher
```

### 方式三：使用参数运行

```bash
# 直接运行并指定参数
python3 wedding_interaction/nodes/realsense_publisher_node.py \
    --ros-args \
    -p topic_name:=/camera/color/image_raw \
    -p fps:=30 \
    -p width:=640 \
    -p height:=480
```

## 验证节点运行

### 1. 检查节点是否启动
```bash
ros2 node list
# 应该看到: /realsense_publisher
```

### 2. 检查 topic 是否发布
```bash
ros2 topic list
# 应该看到: /camera/color/image_raw

# 查看 topic 信息
ros2 topic info /camera/color/image_raw

# 查看图像数据（需要安装 rqt_image_view）
ros2 run rqt_image_view rqt_image_view
```

### 3. 查看节点日志
节点启动后会输出：
- RealSense Pipeline 启动信息
- 发布 topic 名称和频率
- 分辨率信息

## 常见问题

### 1. 无法找到 librealsense2 软件包（E: Unable to locate package）
**问题：** 执行 `sudo apt-get install librealsense2-*` 时提示找不到包。

**快速解决方案（推荐）：** 
对于 Python 节点，通常只需要安装 Python 包即可：
```bash
# 重要：使用 NumPy 1.x 版本
pip3 install "numpy<2" pyrealsense2 opencv-python

# 测试是否可用（简单导入）
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 导入成功')"

# 或者直接测试节点是否能运行（最可靠的方法）
source /opt/ros/<distro>/setup.bash
python3 wedding_interaction/nodes/realsense_publisher_node.py --help
```
如果导入成功，就可以直接运行节点了，不需要系统级的 `librealsense2-*` 包。

**注意：** 
- `pyrealsense2` 模块可能没有 `__version__` 属性，这不影响使用
- **必须使用 NumPy 1.x**（`numpy<2`），因为 `cv_bridge` 不支持 NumPy 2.x

**完整解决方案：** 
如果需要系统工具（如 `realsense-viewer`），需要先添加 Intel 官方软件源（见上方"依赖安装"部分的步骤 3.1）。

**检查软件源是否已添加：**
```bash
# 检查软件源文件是否存在
ls -la /etc/apt/sources.list.d/librealsense.list

# 如果不存在，执行添加软件源的步骤
# 如果存在，检查内容是否正确
cat /etc/apt/sources.list.d/librealsense.list
```

如果添加软件源后仍然找不到包，可能是您的 Ubuntu 版本太新，可以尝试：
- **优先使用 Python 包方案**（见上方快速解决方案）
- 使用源码编译安装 RealSense SDK
- 或使用较旧的 Ubuntu LTS 版本（如 22.04）

### 2. 找不到 RealSense 设备
- 确保 RealSense 相机已连接
- 检查 USB 权限：`sudo chmod 666 /dev/video*`
- 运行 `realsense-viewer` 测试相机是否正常工作
- 检查 USB 连接是否稳定（RealSense 需要 USB 3.0）

### 3. NumPy 版本兼容性问题
**问题：** 出现错误 `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` 或 `AttributeError: _ARRAY_API not found`

**原因：** `cv_bridge` 和 `pyrealsense2` 是用 NumPy 1.x 编译的，不支持 NumPy 2.x

**解决方案：**
```bash
# 卸载当前的 NumPy 2.x
pip3 uninstall numpy

# 安装 NumPy 1.x 版本
pip3 install "numpy<2"

# 验证版本
python3 -c "import numpy; print('NumPy version:', numpy.__version__)"
# 应该显示 1.x 版本（如 1.26.4）
```

### 4. 导入错误
- 确保已安装所有 Python 依赖：`pip3 install "numpy<2" pyrealsense2 opencv-python`
- 检查 ROS2 环境是否正确 source
- 如果 `import pyrealsense2` 失败，可能需要重新安装：`pip3 install --upgrade pyrealsense2`

### 5. 权限问题
如果遇到权限错误，可能需要将用户添加到 video 组：
```bash
sudo usermod -a -G video $USER
# 然后重新登录或执行：
newgrp video
```

## 快速测试脚本

创建测试脚本 `test_realsense.sh`：

```bash
#!/bin/bash
source /opt/ros/<distro>/setup.bash
python3 wedding_interaction/nodes/realsense_publisher_node.py
```

运行：
```bash
chmod +x test_realsense.sh
./test_realsense.sh
```
