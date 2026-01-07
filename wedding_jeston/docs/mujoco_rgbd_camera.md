# MuJoCo 仿真环境 RGBD 相机实现方案

## 1. 概述

本文档描述如何在 MuJoCo 仿真环境中为 PM01 机器人添加 RGBD 相机传感器。由于实际相机（头部和腰部 Intel RealSense）的精确位置尚未确定，我们先在机器人头顶放置一个调试用相机。

### 1.1 当前实现状态 ✅

| 功能 | 状态 | 说明 |
|-----|------|------|
| 模型中添加相机 | ✅ 完成 | `head_rgbd_camera` 已添加到 `LINK_HEAD_YAW` 头顶上方20cm |
| MuJoCo GUI 相机切换 | ✅ 可用 | 在 GUI 中可切换到相机视角 |
| ROS2 话题创建 | ✅ 完成 | `/camera/head/rgb/image_raw` 等话题已创建 |
| 离屏渲染图像发布 | ✅ 完成 | 在渲染线程中实现，640x480 分辨率（RealSense D435i 常用分辨率） |
| PBO 异步传输优化 | ⚠️ 已实现但禁用 | 双缓冲机制代码保留，因稳定性问题暂用同步模式 |
| 黑屏问题修复 | ✅ 完成 | 离屏渲染后恢复窗口缓冲区 |

### 1.2 目标

- ✅ 在机器人头部 (`LINK_HEAD_YAW`) 上方添加 RGBD 相机
- ✅ 实现离屏渲染（Offscreen Rendering）获取相机图像
- ✅ 通过 ROS2 话题发布 RGB 图像和深度图像
- 支持后续扩展到真实相机位置

### 1.2 相关文件

| 文件路径 | 说明 |
|---------|------|
| `mujoco/assets/resource/robot/pm_v2/xml/serial_links.xml` | 机器人连杆定义 |
| `mujoco/assets/resource/robot/pm_v2/xml/serial_sensors.xml` | 传感器定义 |
| `mujoco/assets/config/pm_v2.yaml` | 配置文件 |
| `mujoco/src/ros_interface.cc` | ROS2 接口实现 |
| `mujoco/include/ros_interface.h` | ROS2 接口头文件 |
| `mujoco/src/sim_manager.cc` | 仿真管理器 |

---

## 2. 实现方案

### 2.1 Phase 1: MuJoCo 模型添加相机

#### 2.1.1 修改 `serial_links.xml`

在 `LINK_HEAD_YAW` body 中添加相机定义：

```xml
<!-- 在 LINK_HEAD_YAW body 内添加，位于 line 145-150 附近 -->
<body name="LINK_HEAD_YAW" pos="-0.017638 0 0.2961">
    <inertial pos="0.00358213 0.00030109 0.0882263" quat="0.700279 0.122834 0.123579 0.692278" mass="0.8451" diaginertia="0.00500549 0.00445883 0.00296172"/>
    <joint name="J23_HEAD_YAW" pos="0 0 0" axis="0 0 1" range="-0.6109 0.6109" actuatorfrcrange="-61 61" class="joint" armature="0.039175"/>
    <geom type="mesh" rgba="0.75294 0.75294 0.75294 1" mesh="LINK_HEAD_YAW" class="visual"/>
    <geom class="collision_head"/>
    
    <!-- RGBD Camera - 头顶调试相机 -->
    <!-- pos: x=前, y=左, z=上; 相对于LINK_HEAD_YAW -->
    <!-- xyaxes: 定义相机的x轴和y轴方向 -->
    <camera name="head_rgbd_camera" 
            pos="0.0 0.0 0.2"
            xyaxes="0 -1 0 0 0 1"
            fovy="60"
            mode="fixed"/>
    
    <!-- 相机挂载点 site（用于调试可视化） -->
    <site name="head_camera_site" pos="0.0 0.0 0.2" size="0.02" rgba="1 0 0 0.5"/>
</body>
```

#### 2.1.2 相机参数说明

| 参数 | 值 | 说明 |
|-----|-----|------|
| `pos` | `0.0 0.0 0.2` | 相机位置：头顶上方 20cm |
| `xyaxes` | `0 -1 0 0 0 1` | 相机朝向：向前看（-Y 轴为相机 X，Z 轴为相机 Y） |
| `fovy` | `60` | 垂直视场角（度），模拟 RealSense D435 |
| `mode` | `fixed` | 固定在 body 上 |

#### 2.1.3 Intel RealSense D435i 参考参数

```
实际相机规格（供后续调整参考）：
- RGB: 1920x1080 @ 30fps, FOV 69.4° x 42.5°
- Depth: 1280x720 @ 30fps, FOV 87° x 58°
- 有效深度范围: 0.1m - 10m
```

---

### 2.2 Phase 2: 离屏渲染实现

#### 2.2.1 新增相机渲染类 `camera_renderer.h`

```cpp
// mujoco/include/camera_renderer.h
#ifndef MUJOCO_CAMERA_RENDERER_H_
#define MUJOCO_CAMERA_RENDERER_H_

#include <mujoco/mujoco.h>
#include <memory>
#include <string>
#include <vector>

namespace mujoco {

struct CameraConfig {
  std::string name;           // 相机名称（对应XML中的camera name）
  int width = 640;            // 图像宽度
  int height = 480;           // 图像高度
  float near_clip = 0.01f;    // 近裁剪面 (米)
  float far_clip = 10.0f;     // 远裁剪面 (米)
  double publish_rate = 30.0; // 发布频率 (Hz)
};

class CameraRenderer {
 public:
  CameraRenderer(const CameraConfig& config);
  ~CameraRenderer();

  // 初始化渲染上下文（需要有效的 OpenGL 上下文）
  bool Initialize(const mjModel* model);
  
  // 渲染相机图像
  bool Render(const mjModel* model, const mjData* data);
  
  // 获取RGB图像数据（上下翻转后）
  const std::vector<uint8_t>& GetRgbImage() const { return rgb_buffer_; }
  
  // 获取深度图像数据（单位：米）
  const std::vector<float>& GetDepthImage() const { return depth_buffer_; }
  
  // 获取相机ID
  int GetCameraId() const { return camera_id_; }
  
  // 获取配置
  const CameraConfig& GetConfig() const { return config_; }

 private:
  CameraConfig config_;
  int camera_id_ = -1;
  
  // 渲染上下文
  mjvScene scene_;
  mjvCamera camera_;
  mjvOption option_;
  mjrContext context_;
  
  // 图像缓冲区
  std::vector<uint8_t> rgb_buffer_;    // RGB: width * height * 3
  std::vector<float> depth_buffer_;     // Depth: width * height
  
  bool initialized_ = false;
};

}  // namespace mujoco

#endif  // MUJOCO_CAMERA_RENDERER_H_
```

#### 2.2.2 相机渲染实现 `camera_renderer.cc`

```cpp
// mujoco/src/camera_renderer.cc
#include "camera_renderer.h"
#include <cstring>
#include <iostream>

namespace mujoco {

CameraRenderer::CameraRenderer(const CameraConfig& config) : config_(config) {
  // 预分配缓冲区
  rgb_buffer_.resize(config_.width * config_.height * 3);
  depth_buffer_.resize(config_.width * config_.height);
}

CameraRenderer::~CameraRenderer() {
  if (initialized_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
  }
}

bool CameraRenderer::Initialize(const mjModel* model) {
  if (!model) {
    std::cerr << "CameraRenderer: Invalid model" << std::endl;
    return false;
  }

  // 查找相机ID
  camera_id_ = mj_name2id(model, mjOBJ_CAMERA, config_.name.c_str());
  if (camera_id_ < 0) {
    std::cerr << "CameraRenderer: Camera '" << config_.name << "' not found in model" << std::endl;
    return false;
  }

  // 初始化场景
  mjv_defaultScene(&scene_);
  mjv_makeScene(model, &scene_, 2000);

  // 初始化相机设置
  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;

  // 初始化渲染选项
  mjv_defaultOption(&option_);

  // 创建离屏渲染上下文
  // 注意：需要在有效的 OpenGL 上下文中调用
  mjr_defaultContext(&context_);
  mjr_makeContext(model, &context_, mjFONTSCALE_150);

  // 设置离屏渲染缓冲区大小
  if (!mjr_setBuffer(mjFB_OFFSCREEN, &context_)) {
    std::cerr << "CameraRenderer: Failed to set offscreen buffer" << std::endl;
    return false;
  }

  initialized_ = true;
  std::cout << "CameraRenderer: Initialized camera '" << config_.name 
            << "' (ID: " << camera_id_ << ") with resolution " 
            << config_.width << "x" << config_.height << std::endl;
  return true;
}

bool CameraRenderer::Render(const mjModel* model, const mjData* data) {
  if (!initialized_ || !model || !data) {
    return false;
  }

  // 更新场景
  mjv_updateScene(model, const_cast<mjData*>(data), &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

  // 设置视口
  mjrRect viewport = {0, 0, config_.width, config_.height};

  // 渲染场景到离屏缓冲区
  mjr_render(viewport, &scene_, &context_);

  // 读取RGB图像
  mjr_readPixels(rgb_buffer_.data(), nullptr, viewport, &context_);

  // 读取深度图像
  // MuJoCo返回的是 znear 到 zfar 之间的线性深度
  std::vector<float> raw_depth(config_.width * config_.height);
  mjr_readPixels(nullptr, raw_depth.data(), viewport, &context_);

  // 转换深度值到米
  float extent = model->stat.extent;
  float znear = model->vis.map.znear * extent;
  float zfar = model->vis.map.zfar * extent;
  
  for (int i = 0; i < config_.width * config_.height; ++i) {
    // MuJoCo深度是 [0,1] 归一化值
    if (raw_depth[i] >= 1.0f) {
      depth_buffer_[i] = std::numeric_limits<float>::infinity();
    } else {
      // 转换为实际深度值
      depth_buffer_[i] = znear / (1.0f - raw_depth[i] * (1.0f - znear / zfar));
    }
  }

  // 翻转RGB图像（OpenGL 原点在左下角）
  int row_size = config_.width * 3;
  std::vector<uint8_t> temp_row(row_size);
  for (int y = 0; y < config_.height / 2; ++y) {
    int top_idx = y * row_size;
    int bottom_idx = (config_.height - 1 - y) * row_size;
    std::memcpy(temp_row.data(), &rgb_buffer_[top_idx], row_size);
    std::memcpy(&rgb_buffer_[top_idx], &rgb_buffer_[bottom_idx], row_size);
    std::memcpy(&rgb_buffer_[bottom_idx], temp_row.data(), row_size);
  }

  // 翻转深度图像
  int depth_row_size = config_.width;
  std::vector<float> temp_depth_row(depth_row_size);
  for (int y = 0; y < config_.height / 2; ++y) {
    int top_idx = y * depth_row_size;
    int bottom_idx = (config_.height - 1 - y) * depth_row_size;
    std::memcpy(temp_depth_row.data(), &depth_buffer_[top_idx], depth_row_size * sizeof(float));
    std::memcpy(&depth_buffer_[top_idx], &depth_buffer_[bottom_idx], depth_row_size * sizeof(float));
    std::memcpy(&depth_buffer_[bottom_idx], temp_depth_row.data(), depth_row_size * sizeof(float));
  }

  return true;
}

}  // namespace mujoco
```

---

### 2.3 Phase 3: ROS2 接口扩展

#### 2.3.1 修改 `ros_interface.h`

```cpp
// 在现有 includes 后添加
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "camera_renderer.h"

// 在 RosInterface 类中添加
private:
  // 相机渲染器
  std::unique_ptr<CameraRenderer> camera_renderer_;
  
  // 相机图像发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  
  // 相机发布定时器
  rclcpp::TimerBase::SharedPtr camera_timer_;
  
  // 相机图像发布回调
  void CameraTimerCallback();
  
  // 创建CameraInfo消息
  sensor_msgs::msg::CameraInfo CreateCameraInfoMsg();

public:
  // 初始化相机渲染
  bool InitializeCamera(const mjModel* model);
  
  // 渲染并发布相机图像
  void RenderAndPublishCamera(const mjModel* model, const mjData* data);
```

#### 2.3.2 修改 `ros_interface.cc`

```cpp
// 在 Initialize() 中添加
bool RosInterface::Initialize() {
  // ... 现有代码 ...

  // 创建相机话题发布者
  rgb_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/head/rgb/image_raw", 10);
  depth_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/head/depth/image_raw", 10);
  camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/head/camera_info", 10);

  RCLCPP_INFO(node_->get_logger(), "Camera publishers created");
  return true;
}

bool RosInterface::InitializeCamera(const mjModel* model) {
  // 配置相机参数
  CameraConfig config;
  config.name = "head_rgbd_camera";
  config.width = 640;
  config.height = 480;
  config.publish_rate = 30.0;
  config.near_clip = 0.01f;
  config.far_clip = 10.0f;

  camera_renderer_ = std::make_unique<CameraRenderer>(config);
  if (!camera_renderer_->Initialize(model)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize camera renderer");
    return false;
  }

  // 创建相机发布定时器 (30 Hz)
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / config.publish_rate));
  camera_timer_ = node_->create_wall_timer(
      period, std::bind(&RosInterface::CameraTimerCallback, this));

  RCLCPP_INFO(node_->get_logger(), "Camera initialized: %s (%dx%d @ %.1f Hz)",
              config.name.c_str(), config.width, config.height, config.publish_rate);
  return true;
}

void RosInterface::CameraTimerCallback() {
  if (!camera_renderer_ || !model_ || !data_) {
    return;
  }

  // 渲染图像
  if (!camera_renderer_->Render(model_, data_)) {
    return;
  }

  auto now = node_->now();
  const auto& config = camera_renderer_->GetConfig();

  // 发布 RGB 图像
  {
    auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
    rgb_msg->header.stamp = now;
    rgb_msg->header.frame_id = "head_camera_link";
    rgb_msg->height = config.height;
    rgb_msg->width = config.width;
    rgb_msg->encoding = "rgb8";
    rgb_msg->is_bigendian = false;
    rgb_msg->step = config.width * 3;
    rgb_msg->data = camera_renderer_->GetRgbImage();
    rgb_image_pub_->publish(std::move(rgb_msg));
  }

  // 发布深度图像
  {
    auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg->header.stamp = now;
    depth_msg->header.frame_id = "head_camera_link";
    depth_msg->height = config.height;
    depth_msg->width = config.width;
    depth_msg->encoding = "32FC1";  // 32位浮点深度
    depth_msg->is_bigendian = false;
    depth_msg->step = config.width * sizeof(float);
    
    const auto& depth_data = camera_renderer_->GetDepthImage();
    depth_msg->data.resize(depth_data.size() * sizeof(float));
    std::memcpy(depth_msg->data.data(), depth_data.data(), depth_msg->data.size());
    depth_image_pub_->publish(std::move(depth_msg));
  }

  // 发布相机信息
  camera_info_pub_->publish(CreateCameraInfoMsg());
}

sensor_msgs::msg::CameraInfo RosInterface::CreateCameraInfoMsg() {
  const auto& config = camera_renderer_->GetConfig();
  sensor_msgs::msg::CameraInfo info;
  
  info.header.stamp = node_->now();
  info.header.frame_id = "head_camera_link";
  info.height = config.height;
  info.width = config.width;
  info.distortion_model = "plumb_bob";
  
  // 计算焦距（基于 fovy）
  double fovy_rad = 60.0 * M_PI / 180.0;  // 与XML中的fovy对应
  double fy = config.height / (2.0 * std::tan(fovy_rad / 2.0));
  double fx = fy;  // 假设正方形像素
  double cx = config.width / 2.0;
  double cy = config.height / 2.0;
  
  // 内参矩阵 K
  info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
  
  // 畸变系数 D (无畸变)
  info.d = {0, 0, 0, 0, 0};
  
  // 矫正矩阵 R (单位矩阵)
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  
  // 投影矩阵 P
  info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
  
  return info;
}
```

#### 2.3.3 修改 `sim_manager.cc`

```cpp
// 在 PhysicsThread() 中，模型加载后初始化相机
void SimManager::PhysicsThread(std::string_view filename) {
  // ... 现有代码 ...
  
  if (d_) {
    sim_->Load(m_, d_, filename.data());
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mj_forward(m_, d_);
    
    // 初始化相机渲染器
    if (ros_interface_) {
      if (!ros_interface_->InitializeCamera(m_)) {
        RCLCPP_WARN(logger, "Camera initialization failed, continuing without camera");
      }
    }
  }
  
  // ... 剩余代码 ...
}
```

---

### 2.4 Phase 4: CMakeLists.txt 更新

```cmake
# 添加 sensor_msgs 依赖
find_package(sensor_msgs REQUIRED)

# 更新源文件列表
add_executable(mujoco_simulator
  src/main.cc
  src/ros_interface.cc
  src/config_loader.cc
  src/sim_manager.cc
  src/camera_renderer.cc  # 新增
)

# 更新 ament 依赖
ament_target_dependencies(mujoco_simulator
  rclcpp
  interface_protocol
  std_msgs
  geometry_msgs
  sensor_msgs  # 新增
)
```

---

### 2.5 Phase 5: 配置文件更新

#### 2.5.1 修改 `pm_v2.yaml`

```yaml
urdf: serial_pm_v2.urdf
xml: pm_v2.xml

model_param:
  num_total_joints: 24
  num_contacts: 4
  num_single_contact_dimensions: 2

sensor:
  imu_topic: /hardware/imu_info

actuator:
  joint_state_topic: /hardware/joint_state
  joint_command_topic: /hardware/joint_command

# 新增相机配置
camera:
  head:
    name: head_rgbd_camera
    width: 640
    height: 480
    fps: 30
    rgb_topic: /camera/head/rgb/image_raw
    depth_topic: /camera/head/depth/image_raw
    info_topic: /camera/head/camera_info
    frame_id: head_camera_link
```

---

## 3. 测试验证

### 3.1 编译

```bash
cd ~/project/engine_ai/engineai_ros2_workspace
colcon build --packages-select mujoco_simulator --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3.2 启动仿真

```bash
ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
```

### 3.3 在 MuJoCo GUI 中查看相机视角 ✅

相机已成功添加到模型中。在 MuJoCo GUI 中切换到相机视角：

1. 在 MuJoCo 窗口右侧面板中找到 **Rendering** 部分
2. 点击 **Camera** 下拉菜单
3. 选择 **head_rgbd_camera**
4. 现在视角会切换到机器人头顶的相机位置

验证日志输出：
```
[mujoco_physics]: RGBD camera 'head_rgbd_camera' found in model (ID: 0)
[mujoco_physics]: You can view from this camera in MuJoCo GUI: Rendering > Camera > head_rgbd_camera
```

### 3.4 查看相机话题

```bash
# 查看话题列表
ros2 topic list | grep camera

# 预期输出:
# /camera/head/rgb/image_raw
# /camera/head/depth/image_raw
# /camera/head/camera_info
```

**注意**: 当前版本话题已创建，但离屏渲染尚未完成实现，因此话题暂无图像数据。

### 3.5 使用 RViz2 可视化（离屏渲染完成后）

```bash
# 启动 RViz2
rviz2

# 添加 Image 显示:
# - Topic: /camera/head/rgb/image_raw
# - Topic: /camera/head/depth/image_raw
```

### 3.6 使用 rqt_image_view（离屏渲染完成后）

```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /camera/head/rgb/image_raw
```

---

## 4. 后续扩展

### 4.1 添加真实相机位置

当获得真实相机的精确位置后，在 `serial_links.xml` 中添加：

```xml
<!-- 头部相机（实际位置） -->
<camera name="head_camera_real" 
        pos="0.05 0 0.08"  <!-- 根据实际测量调整 -->
        xyaxes="0 -1 0 0 0 1"
        fovy="69.4"/>  <!-- RealSense D435 RGB FOV -->

<!-- 腰部相机 -->
<body name="LINK_TORSO_YAW" ...>
    <camera name="waist_camera"
            pos="0.1 0 0.15"  <!-- 根据实际测量调整 -->
            xyaxes="0 -1 0 0 0 1"
            fovy="69.4"/>
</body>
```

### 4.2 多相机支持

修改 `CameraRenderer` 支持多相机实例：

```cpp
std::vector<std::unique_ptr<CameraRenderer>> camera_renderers_;

// 遍历所有配置的相机
for (const auto& cam_config : camera_configs) {
  auto renderer = std::make_unique<CameraRenderer>(cam_config);
  if (renderer->Initialize(model)) {
    camera_renderers_.push_back(std::move(renderer));
  }
}
```

### 4.3 点云生成

添加 PointCloud2 发布：

```cpp
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// 从深度图生成点云
sensor_msgs::msg::PointCloud2 CreatePointCloud(
    const std::vector<float>& depth,
    const std::vector<uint8_t>& rgb,
    const CameraConfig& config);
```

---

## 5. 性能分析与优化

### 5.1 性能瓶颈分析

开启相机后 MuJoCo 变卡的主要原因：

| 瓶颈 | 影响程度 | 说明 |
|-----|---------|------|
| **双重渲染** | ⭐⭐⭐ 高 | 主窗口 + 相机离屏，场景渲染两次 |
| **像素读取阻塞** | ⭐⭐⭐ 高 | `mjr_readPixels` 等待 GPU 完成，造成 GPU stall |
| **大量数据传输** | ⭐⭐ 中 | 每帧 ~2MB 数据从 GPU→CPU（RGB + Depth） |
| **CPU 后处理** | ⭐ 低 | 深度转换、图像翻转 |

### 5.2 MuJoCo 的 GPU 使用

| 组件 | 使用 GPU? | 说明 |
|------|---------|------|
| 物理仿真 | ❌ CPU | 刚体动力学、碰撞检测、接触求解 |
| 场景渲染 | ✅ GPU | 使用 OpenGL 渲染 3D 场景 |
| 像素读取 | ⚠️ 同步 | GPU→CPU 传输，会阻塞渲染流水线 |

**结论**：卡顿不是电脑配置问题，而是**同步像素读取**导致 GPU 和 CPU 互相等待。

### 5.3 已实施的优化

1. **帧率限制**：从 30Hz 降到 10Hz（可调节）
   ```cpp
   // ros_interface.cc
   constexpr int64_t CAMERA_PUBLISH_INTERVAL_MS = 100;  // 10 Hz
   ```

2. **避免重复内存分配**：深度缓冲区预分配，避免每帧 `new/delete`

3. **合并像素读取**：RGB 和深度一次调用读取

4. **预计算常量**：深度转换的常量移到循环外

5. **PBO 双缓冲异步传输** ⚠️ 已实现但暂时禁用
   - 使用 OpenGL Pixel Buffer Object (PBO) 实现 GPU→CPU 异步数据传输
   - 双缓冲机制：GPU 写入一个 PBO 时，CPU 从另一个读取
   - 消除 GPU stall，GPU 和 CPU 可并行工作
   - 通过 `glfwGetProcAddress` 动态加载 OpenGL 扩展函数
   - **当前状态**：因与共享 OpenGL 上下文冲突导致稳定性问题，默认禁用
   - 如需启用：修改 `camera_renderer.h` 中 `use_pbo = true`

6. **黑屏问题修复** ✅
   - 离屏渲染后调用 `mjr_setBuffer(mjFB_WINDOW, &context_)` 恢复窗口缓冲区
   - 避免主窗口显示黑屏

### 5.4 进一步优化建议

**方案 1: 降低分辨率**（如需要）
```cpp
// ros_interface.cc InitializeCameraRenderer() 中修改
config.width = 320;   // 从 640 降到 320
config.height = 240;  // 从 480 降到 240
// 数据量减少 75%！
```

**方案 2: 降低发布频率**
```cpp
// ros_interface.cc 中修改
constexpr int64_t CAMERA_PUBLISH_INTERVAL_MS = 200;  // 5 Hz
// 或
constexpr int64_t CAMERA_PUBLISH_INTERVAL_MS = 500;  // 2 Hz
```

**方案 3: 只发布 RGB，禁用深度**
```cpp
// 在 camera_renderer.cc 的 Render() 中
// 注释掉深度读取相关代码
```

**方案 4: PBO 异步渲染** ⚠️ 已实现但暂时禁用
- 使用 PBO (Pixel Buffer Object) 进行异步像素读取
- 实现了双缓冲（double-buffering）机制
- **当前状态**：因与共享 OpenGL 上下文冲突导致程序崩溃，默认禁用

PBO 优化原理：
```
传统同步模式：
  GPU渲染 → GPU等待 → CPU读取 → CPU处理 → 下一帧
           ↑ 阻塞！

PBO双缓冲模式：
  帧N: GPU渲染 → 异步读取到PBO[0] ─────────────────┐
  帧N: 同时从PBO[1]读取帧N-1数据 → CPU处理 → 发布  │
                                                    ↓
  帧N+1: GPU渲染 → 异步读取到PBO[1] ←──────────────┘
  帧N+1: 同时从PBO[0]读取帧N数据 → CPU处理 → 发布
```

代码实现（`camera_renderer.cc`）：
```cpp
// OpenGL PBO 函数通过 glfwGetProcAddress 动态加载
static PFNGLGENBUFFERSPROC pglGenBuffers = nullptr;
static PFNGLBINDBUFFERPROC pglBindBuffer = nullptr;
static PFNGLMAPBUFFERPROC pglMapBuffer = nullptr;
// ...

// 双缓冲 PBO
unsigned int pbo_rgb_[2];   // ping-pong RGB buffers
unsigned int pbo_depth_[2]; // ping-pong depth buffers
int pbo_index_ = 0;         // 当前缓冲区索引

bool CameraRenderer::RenderWithPBO(const mjModel* model, const mjData* data) {
    int current = pbo_index_;
    int previous = (pbo_index_ + 1) % 2;
    
    // 1. 启动异步读取到当前PBO
    pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_rgb_[current]);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    
    // 2. 从前一帧的PBO读取数据（非阻塞）
    pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_rgb_[previous]);
    void* data = pglMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    // ... 处理数据
    
    pbo_index_ = (pbo_index_ + 1) % 2; // 切换缓冲区
}
```

启用方法（如需测试）：
```cpp
// camera_renderer.h 中修改
bool use_pbo = true;  // 默认为 false
```

日志输出（启用 PBO 时）：
```
[CameraRenderer] PBO async transfer enabled (double-buffering)
[CameraRenderer] Initialized camera 'head_rgbd_camera' (ID: 0) with resolution 640x480 [PBO async]
```

日志输出（同步模式，当前默认）：
```
[CameraRenderer] Initialized camera 'head_rgbd_camera' (ID: 0) with resolution 640x480 [sync]
```

### 5.5 查看相机图像

```bash
# 方法1: rqt_image_view
ros2 run rqt_image_view rqt_image_view

# 方法2: image_view  
ros2 run image_view image_view --ros-args -r image:=/camera/head/rgb/image_raw

# 方法3: rviz2
rviz2
# 添加 Image 显示，选择话题 /camera/head/rgb/image_raw
```

### 5.6 性能调优参数表

| 参数 | 位置 | 默认值 | 调整建议 |
|-----|------|-------|---------|
| 分辨率 | `ros_interface.cc` InitializeCameraRenderer() | 640×480 | 可降到 320×240 |
| 发布频率 | `ros_interface.cc` config.publish_rate | 30 Hz | 可降到 10-15 Hz |
| 视场角 | `serial_links.xml` camera fovy | 60° | 按需调整 |
| PBO 模式 | `camera_renderer.h` use_pbo | false | 如需要可改为 true（可能不稳定） |

---

## 6. 文件变更总结

| 操作 | 文件 |
|-----|------|
| 修改 | `mujoco/assets/resource/robot/pm_v2/xml/serial_links.xml` |
| 修改 | `mujoco/assets/config/pm_v2.yaml` |
| 新增 | `mujoco/include/camera_renderer.h` |
| 新增 | `mujoco/src/camera_renderer.cc` |
| 修改 | `mujoco/include/ros_interface.h` |
| 修改 | `mujoco/src/ros_interface.cc` |
| 修改 | `mujoco/src/sim_manager.cc` |
| 修改 | `mujoco/CMakeLists.txt` |

---

## 7. 参考资料

- [MuJoCo Documentation - Cameras](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera)
- [MuJoCo Rendering](https://mujoco.readthedocs.io/en/stable/programming/rendering.html)
- [ROS2 sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/index.html)
- [Intel RealSense D435i Specs](https://www.intelrealsense.com/depth-camera-d435i/)

