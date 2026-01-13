#ifndef MUJOCO_ROS_INTERFACE_H_
#define MUJOCO_ROS_INTERFACE_H_

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "rclcpp/rclcpp.hpp"

// Camera-related includes
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "camera_renderer.h"

// MuJoCo includes
#include <mujoco/mujoco.h>

// Forward declarations
namespace mujoco {
class Simulate;
}
class ConfigLoader;

namespace mujoco {

class RosInterface {
 public:
  RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader);
  ~RosInterface();

  // Initialize the MuJoCo interface
  bool Initialize();

  // Callback for joint command messages
  void JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg);

  // Update the simulation state to publish to ROS
  void UpdateSimState(const mjModel* m, mjData* d);

  // Get joint command values (thread-safe)
  interface_protocol::msg::JointCommand GetCommandedSafe();
  
  // Check if first valid command has been received
  bool HasReceivedFirstCommand() const { return received_first_command_; }

  // Set the current mjModel and mjData
  void SetModelAndData(mjModel* model, mjData* data);

  // Get the ROS node
  rclcpp::Node::SharedPtr GetNode() const { return node_; }

  // Initialize camera renderer (call after model is loaded, from render thread)
  // Uses the shared mjrContext from the main window to ensure textures are available
  bool InitializeCamera(const mjModel* model, mjrContext* shared_context);

  // Render and publish camera images (call from render thread with valid context)
  void RenderAndPublishCamera(const mjModel* model, const mjData* data);

 private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<interface_protocol::msg::ImuInfo>::SharedPtr imu_pub_;
  rclcpp::Publisher<interface_protocol::msg::MotionState>::SharedPtr motion_state_pub_;

  // Subscribers
  rclcpp::Subscription<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr photo_panel_sub_;

  // Config loader
  std::shared_ptr<ConfigLoader> config_loader_;

  // Number of joints
  int num_total_joints_ = 0;

  // Current joint command
  interface_protocol::msg::JointCommand joint_command_;

  // MuJoCo model and data
  mjModel* model_;
  mjData* data_;

  // Timer for publishing motion state
  rclcpp::TimerBase::SharedPtr motion_state_timer_;
  
  // Motion state timer callback
  void MotionStateTimerCallback();
  
  // Timer for periodic joint state publishing (when sim is not running)
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  
  // Joint state timer callback
  void JointStateTimerCallback();

  // Mutex for thread safety
  std::mutex mtx_;

  // Flag indicating if we have a floating base robot
  bool is_floating_base_;
  
  // Flag indicating if we have received the first valid command
  bool received_first_command_ = false;

  // Flag indicating if we should only run actions (no publishing)
  bool only_action_ = false;

  // ===== Camera-related members =====
  // Camera renderer
  std::unique_ptr<CameraRenderer> camera_renderer_;

  // Camera image publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Camera publishing timer
  rclcpp::TimerBase::SharedPtr camera_timer_;

  // Last render time for rate limiting
  std::chrono::steady_clock::time_point last_camera_render_time_;

  // Camera timer callback
  void CameraTimerCallback();

  // Create camera info message
  sensor_msgs::msg::CameraInfo CreateCameraInfoMsg() const;

  // Flag indicating if camera is initialized
  bool camera_initialized_ = false;

  // ===== Photo panel control =====
  // Callback for photo panel position
  void PhotoPanelPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  
  // Photo panel target position
  geometry_msgs::msg::Point photo_panel_position_;
  bool photo_panel_position_updated_ = false;
  int photo_panel_body_id_ = -1;

 public:
  // Update photo panel position in simulation (called from physics loop)
  void UpdatePhotoPanelPosition();
};

}  // namespace mujoco

#endif  // MUJOCO_ROS_INTERFACE_H_
