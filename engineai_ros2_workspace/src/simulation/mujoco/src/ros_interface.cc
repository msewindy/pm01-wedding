#include "ros_interface.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>

#include "config_loader.h"
#include "rclcpp/rclcpp.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constants
const int kDofFloatingBase = 6;        // Number of DoF for floating base
const int kNumFloatingBaseJoints = 7;  // Number of joints for floating base (quaternion + xyz)
const int kDimQuaternion = 4;          // Dimension of a quaternion

namespace mujoco {

RosInterface::RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader)
    : node_(node), config_loader_(config_loader), model_(nullptr), data_(nullptr), is_floating_base_(false) {}

RosInterface::~RosInterface() {}

bool RosInterface::Initialize() {
  // Check parameters
  node_->declare_parameter("only_action", false);
  only_action_ = node_->get_parameter("only_action").as_bool();
  
  // Create subscriber with more compatible QoS settings
  using std::placeholders::_1;
  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();

  if (only_action_) {
    RCLCPP_WARN(node_->get_logger(), 
        "PARAMETER 'only_action' IS TRUE. ROS PUBLISHERS ARE DISABLED.");
  } else {
    // Create publishers only if only_action is false
    joint_state_pub_ =
        node_->create_publisher<interface_protocol::msg::JointState>(config_loader_->GetJointStateTopic(), qos);
    RCLCPP_INFO(node_->get_logger(), "Joint state publisher created on topic: %s", 
                config_loader_->GetJointStateTopic().c_str());

    imu_pub_ = node_->create_publisher<interface_protocol::msg::ImuInfo>(config_loader_->GetImuTopic(), qos);
    RCLCPP_INFO(node_->get_logger(), "IMU publisher created on topic: %s", 
                config_loader_->GetImuTopic().c_str());
    
    // Create publisher for motion state
    motion_state_pub_ = node_->create_publisher<interface_protocol::msg::MotionState>("/motion/motion_state", 10);
    
    // Create camera publishers
    rgb_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/head/rgb/image_raw", qos);
    depth_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/head/depth/image_raw", qos);
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/head/camera_info", qos);
    
    RCLCPP_INFO(node_->get_logger(), "Camera publishers created (/camera/head/*)");
  }

  // 创建订阅者使用的QoS（KeepLast(1)用于命令订阅）
  rclcpp::QoS cmd_qos(1);
  cmd_qos.best_effort();
  cmd_qos.durability_volatile();
  
  joint_cmd_sub_ = node_->create_subscription<interface_protocol::msg::JointCommand>(
      config_loader_->GetJointCommandTopic(), cmd_qos, std::bind(&RosInterface::JointCommandCallback, this, _1));
  RCLCPP_INFO(node_->get_logger(), "Joint command subscriber created on topic: %s", 
              config_loader_->GetJointCommandTopic().c_str());

  // Get number of joints from config loader
  num_total_joints_ = config_loader_->GetNumTotalJoints();

  // Initialize commanded values with safe defaults
  // 使用高刚度默认值，防止机器人在收到控制命令前倒下
  joint_command_.position.resize(num_total_joints_, 0.0);
  joint_command_.velocity.resize(num_total_joints_, 0.0);
  joint_command_.torque.resize(num_total_joints_, 0.0);
  joint_command_.feed_forward_torque.resize(num_total_joints_, 0.0);
  joint_command_.stiffness.resize(num_total_joints_, 400.0);  // 默认高刚度
  joint_command_.damping.resize(num_total_joints_, 5.0);      // 默认阻尼
  
  // 标记：等待第一条有效命令
  received_first_command_ = false;

  // Create timer for publishing motion state every 1 second
  motion_state_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RosInterface::MotionStateTimerCallback, this));
  
  // Create timer for periodic joint state publishing (100Hz = 10ms)
  // This ensures joint state is published even when simulation is paused
  // Note: Inside callback, it checks for null publishers if only_action is true
  joint_state_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&RosInterface::JointStateTimerCallback, this));

  // Create photo panel position subscriber (Always created)
  photo_panel_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
      "/sim/photo_panel/position", 10,
      std::bind(&RosInterface::PhotoPanelPositionCallback, this, _1));
  RCLCPP_INFO(node_->get_logger(), "Photo panel position subscriber created");

  // Spin a few times to ensure publishers are registered with ROS2 discovery
  // This is important for ROS2 DDS discovery mechanism
  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  RCLCPP_INFO(node_->get_logger(), "MuJoCo ROS interface initialized successfully");
  if (joint_state_pub_) {
      RCLCPP_INFO(node_->get_logger(), "Joint state publisher subscription count: %zu", 
                  joint_state_pub_->get_subscription_count());
  }
  return true;
}

interface_protocol::msg::JointCommand RosInterface::GetCommandedSafe() {
  std::lock_guard<std::mutex> lock(mtx_);
  return joint_command_;
}

void RosInterface::JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  // 标记收到第一条命令
  if (!received_first_command_) {
    received_first_command_ = true;
    RCLCPP_INFO(node_->get_logger(), "[DEBUG] First joint command received!");
  }

  // Debug log - 每 500 次打印一次
  static int callback_count = 0;
  callback_count++;
  if (callback_count % 500 == 1) {
    RCLCPP_INFO(node_->get_logger(), 
      "[DEBUG] Received joint_command: pos[0]=%.4f, stiffness[0]=%.1f, pos_size=%zu",
      msg->position.size() > 0 ? msg->position[0] : -999.0,
      msg->stiffness.size() > 0 ? msg->stiffness[0] : -999.0,
      msg->position.size());
  }

  // Update commanded values
  joint_command_ = *msg;

  // Ensure all vectors are properly sized
  if (joint_command_.position.size() > num_total_joints_) {
    joint_command_.position.resize(num_total_joints_);
  }
  if (joint_command_.velocity.size() > num_total_joints_) {
    joint_command_.velocity.resize(num_total_joints_);
  }
  if (joint_command_.torque.size() > num_total_joints_) {
    joint_command_.torque.resize(num_total_joints_);
  }
  if (joint_command_.feed_forward_torque.size() > num_total_joints_) {
    joint_command_.feed_forward_torque.resize(num_total_joints_);
  }
  if (joint_command_.stiffness.size() > num_total_joints_) {
    joint_command_.stiffness.resize(num_total_joints_);
  }
  if (joint_command_.damping.size() > num_total_joints_) {
    joint_command_.damping.resize(num_total_joints_);
  }
}

void RosInterface::UpdateSimState(const mjModel* m, mjData* d) {
  // Check if interface is enabled (publishers initialized)
  if (!joint_state_pub_ || !imu_pub_) {
    return;
  }

  is_floating_base_ = (m->nv != m->nu);

  // Create messages
  auto joint_state_msg = std::make_unique<interface_protocol::msg::JointState>();
  auto imu_msg = std::make_unique<interface_protocol::msg::ImuInfo>();

  // Set timestamp
  joint_state_msg->header.stamp = node_->now();
  imu_msg->header.stamp = node_->now();

  // Set joint states
  joint_state_msg->position.resize(num_total_joints_);
  joint_state_msg->velocity.resize(num_total_joints_);
  joint_state_msg->torque.resize(num_total_joints_);

  if (is_floating_base_) {
    // Skip the floating base joints
    for (int i = 0; i < num_total_joints_; ++i) {
      joint_state_msg->position[i] = d->qpos[i + kNumFloatingBaseJoints];
      joint_state_msg->velocity[i] = d->qvel[i + kDofFloatingBase];
      joint_state_msg->torque[i] = d->actuator_force[i];
    }
  } else {
    for (int i = 0; i < num_total_joints_; ++i) {
      joint_state_msg->position[i] = d->qpos[i];
      joint_state_msg->velocity[i] = d->qvel[i];
      joint_state_msg->torque[i] = d->actuator_force[i];
    }
  }

  // IMU data typically comes from sensors in MuJoCo
  int index = 0;

  // Set IMU quaternion
  imu_msg->quaternion.w = d->sensordata[index + 0];
  imu_msg->quaternion.x = d->sensordata[index + 1];
  imu_msg->quaternion.y = d->sensordata[index + 2];
  imu_msg->quaternion.z = d->sensordata[index + 3];
  index += kDimQuaternion;

  // Set RPY values from the sensor data
  // Assuming the RPY values are the next three values after the quaternion
  imu_msg->rpy.x = d->sensordata[index + 0];  // Roll
  imu_msg->rpy.y = d->sensordata[index + 1];  // Pitch
  imu_msg->rpy.z = d->sensordata[index + 2];  // Yaw
  index += 3;

  // Linear acceleration
  imu_msg->linear_acceleration.x = d->sensordata[index + 0];
  imu_msg->linear_acceleration.y = d->sensordata[index + 1];
  imu_msg->linear_acceleration.z = d->sensordata[index + 2];
  index += 3;

  // Angular velocity
  imu_msg->angular_velocity.x = d->sensordata[index + 0];
  imu_msg->angular_velocity.y = d->sensordata[index + 1];
  imu_msg->angular_velocity.z = d->sensordata[index + 2];

  // Publish messages
  static int publish_count = 0;
  publish_count++;
  
  if (publish_count % 1000 == 1) {  // 每1000次打印一次（约每秒一次 @ 1000Hz）
    RCLCPP_INFO(node_->get_logger(), 
                "[DEBUG] Publishing joint state: %zu joints, topic: %s, subscribers: %zu",
                joint_state_msg->position.size(),
                config_loader_->GetJointStateTopic().c_str(),
                joint_state_pub_->get_subscription_count());
  }
  
  // Publish messages (will be queued even if no subscribers)
  joint_state_pub_->publish(std::move(joint_state_msg));
  imu_pub_->publish(std::move(imu_msg));
}

void RosInterface::SetModelAndData(mjModel* model, mjData* data) {
  model_ = model;
  data_ = data;
}

void RosInterface::MotionStateTimerCallback() {
  if (!motion_state_pub_) return;

  // Create a motion state message
  auto motion_state_msg = std::make_unique<interface_protocol::msg::MotionState>();
  
  // Set the current_motion_task field to "joint_bridge"
  motion_state_msg->current_motion_task = "joint_bridge";
  
  // Publish the message
  motion_state_pub_->publish(std::move(motion_state_msg));
}

void RosInterface::JointStateTimerCallback() {
  // Only publish if model and data are set
  if (model_ && data_) {
    UpdateSimState(model_, data_);
  }
}

bool RosInterface::InitializeCamera(const mjModel* model, mjrContext* shared_context) {
  if (!model) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot initialize camera: model is null");
    return false;
  }

  // Check if camera already initialized
  if (camera_initialized_) {
    return true;
  }

  // Check if camera exists in model
  int cam_id = mj_name2id(model, mjOBJ_CAMERA, "head_rgbd_camera");
  if (cam_id < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Camera 'head_rgbd_camera' not found in model");
    return false;
  }

  // Configure camera parameters
  // NOTE: Lower resolution = better performance
  // 640x480: ~4Hz (卡顿), 320x240: ~15Hz (流畅), 160x120: ~30Hz (最流畅)
  CameraConfig config;
  config.name = "head_rgbd_camera";
  config.width = 640;   // RealSense D435i 常用分辨率
  config.height = 480;
  config.publish_rate = 30.0;
  config.near_clip = 0.01f;
  config.far_clip = 10.0f;

  camera_renderer_ = std::make_unique<CameraRenderer>(config);
  
  // Initialize with shared context to reuse loaded textures
  // This is important: creating a new context would lose all textures!
  if (!camera_renderer_->Initialize(model, shared_context)) {
    RCLCPP_WARN(node_->get_logger(), "Camera renderer initialization failed, will retry");
    camera_renderer_.reset();
    return false;
  }

  camera_initialized_ = true;
  last_camera_render_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(), "Camera initialized: %s (%dx%d)",
              config.name.c_str(), config.width, config.height);
  return true;
}

void RosInterface::CameraTimerCallback() {
  if (only_action_ || !camera_initialized_ || !camera_renderer_ || !model_ || !data_) {
    return;
  }

  // Render and publish camera images
  RenderAndPublishCamera(model_, data_);
}

void RosInterface::RenderAndPublishCamera(const mjModel* model, const mjData* data) {
  if (!camera_initialized_ || !camera_renderer_ || !camera_renderer_->IsInitialized()) {
    return;
  }

  // Rate limiting: only publish at ~10 Hz (every ~100ms) to reduce performance impact
  // Adjust this value based on your needs: 100ms = 10Hz, 200ms = 5Hz, 500ms = 2Hz
  constexpr int64_t CAMERA_PUBLISH_INTERVAL_MS = 100;  // 10 Hz
  
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      current_time - last_camera_render_time_).count();
  if (elapsed < CAMERA_PUBLISH_INTERVAL_MS) {
    return;
  }
  last_camera_render_time_ = current_time;

  // Render images
  if (!camera_renderer_->Render(model, data)) {
    static int error_count = 0;
    if (++error_count % 100 == 1) {
      RCLCPP_WARN(node_->get_logger(), "Camera render failed (count: %d)", error_count);
    }
    return;
  }

  auto now = node_->now();
  const auto& config = camera_renderer_->GetConfig();

  
  // Publish RGB image
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

  // Publish depth image
  {
    auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
    depth_msg->header.stamp = now;
    depth_msg->header.frame_id = "head_camera_link";
    depth_msg->height = config.height;
    depth_msg->width = config.width;
    depth_msg->encoding = "32FC1";  // 32-bit float depth
    depth_msg->is_bigendian = false;
    depth_msg->step = config.width * sizeof(float);

    const auto& depth_data = camera_renderer_->GetDepthImage();
    depth_msg->data.resize(depth_data.size() * sizeof(float));
    std::memcpy(depth_msg->data.data(), depth_data.data(), depth_msg->data.size());
    depth_image_pub_->publish(std::move(depth_msg));
  }

  // Publish camera info
  camera_info_pub_->publish(CreateCameraInfoMsg());
  

  // Debug log every 5 seconds
  static auto last_log_time = std::chrono::steady_clock::now();
  auto log_check_time = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(log_check_time - last_log_time).count() >= 5) {
    RCLCPP_INFO(node_->get_logger(), "Camera publishing: RGB and Depth images @ 30 Hz");
    last_log_time = log_check_time;
  }
}

sensor_msgs::msg::CameraInfo RosInterface::CreateCameraInfoMsg() const {
  const auto& config = camera_renderer_->GetConfig();
  sensor_msgs::msg::CameraInfo info;

  info.header.stamp = node_->now();
  info.header.frame_id = "head_camera_link";
  info.height = config.height;
  info.width = config.width;
  info.distortion_model = "plumb_bob";

  // Calculate focal length based on fovy (60 degrees, matching XML)
  // MuJoCo only supports fovy in XML, and calculates fovx automatically from fovy and aspect ratio
  double fovy_rad = 60.0 * M_PI / 180.0;
  double fy = config.height / (2.0 * std::tan(fovy_rad / 2.0));
  
  // Calculate fovx from fovy and aspect ratio (matching MuJoCo's rendering behavior)
  double aspect_ratio = static_cast<double>(config.width) / config.height;
  double fovx_rad = 2.0 * std::atan(aspect_ratio * std::tan(fovy_rad / 2.0));
  double fx = config.width / (2.0 * std::tan(fovx_rad / 2.0));
  
  double cx = config.width / 2.0;
  double cy = config.height / 2.0;

  // Intrinsic matrix K (3x3, row-major)
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};

  // Distortion coefficients D (no distortion)
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Rectification matrix R (identity)
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  // Projection matrix P (3x4, row-major)
  info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

  return info;
}

void RosInterface::PhotoPanelPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  photo_panel_position_ = *msg;
  photo_panel_position_updated_ = true;
  
  static int callback_count = 0;
  if (++callback_count % 10 == 1) {
    RCLCPP_INFO(node_->get_logger(), 
      "Photo panel position: x=%.2f, y=%.2f, z=%.2f",
      msg->x, msg->y, msg->z);
  }
}

void RosInterface::UpdatePhotoPanelPosition() {
  if (!photo_panel_position_updated_ || model_ == nullptr || data_ == nullptr) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(mtx_);
  
  // Find photo_panel body if not cached
  if (photo_panel_body_id_ < 0) {
    photo_panel_body_id_ = mj_name2id(model_, mjOBJ_BODY, "photo_panel");
    if (photo_panel_body_id_ < 0) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "Body 'photo_panel' not found in model");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Found photo_panel body at id=%d", photo_panel_body_id_);
  }
  
  // Get the joint id for this body (assuming it's a free joint or has mocap)
  int jnt_id = model_->body_jntadr[photo_panel_body_id_];
  
  if (jnt_id >= 0 && model_->jnt_type[jnt_id] == mjJNT_FREE) {
    // Free joint: first 3 qpos entries are position (x, y, z)
    int qpos_adr = model_->jnt_qposadr[jnt_id];
    data_->qpos[qpos_adr + 0] = photo_panel_position_.x;
    data_->qpos[qpos_adr + 1] = photo_panel_position_.y;
    data_->qpos[qpos_adr + 2] = photo_panel_position_.z;
  } else {
    // Try mocap body
    int mocap_id = model_->body_mocapid[photo_panel_body_id_];
    if (mocap_id >= 0) {
      data_->mocap_pos[3 * mocap_id + 0] = photo_panel_position_.x;
      data_->mocap_pos[3 * mocap_id + 1] = photo_panel_position_.y;
      data_->mocap_pos[3 * mocap_id + 2] = photo_panel_position_.z;
    } else {
      // Static body - need to modify body position directly (not recommended for physics sim)
      // For visualization-only bodies, we can try modifying xpos
      data_->xpos[3 * photo_panel_body_id_ + 0] = photo_panel_position_.x;
      data_->xpos[3 * photo_panel_body_id_ + 1] = photo_panel_position_.y;
      data_->xpos[3 * photo_panel_body_id_ + 2] = photo_panel_position_.z;
    }
  }
  
  photo_panel_position_updated_ = false;
}

}  // namespace mujoco
