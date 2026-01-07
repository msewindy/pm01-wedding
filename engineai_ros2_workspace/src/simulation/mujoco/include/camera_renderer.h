// Copyright 2026 EngineAI Robotics
// RGBD Camera Renderer for MuJoCo Simulation
// Optimized with PBO (Pixel Buffer Object) for async GPU->CPU transfer

#ifndef MUJOCO_CAMERA_RENDERER_H_
#define MUJOCO_CAMERA_RENDERER_H_

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <memory>
#include <string>
#include <vector>

namespace mujoco {

/// @brief Camera configuration parameters
struct CameraConfig {
  std::string name = "head_rgbd_camera";  // Camera name in MuJoCo model
  int width = 640;                         // Image width in pixels
  int height = 480;                        // Image height in pixels
  float near_clip = 0.01f;                 // Near clipping plane (meters)
  float far_clip = 10.0f;                  // Far clipping plane (meters)
  double publish_rate = 30.0;              // Publishing rate (Hz)
  bool use_pbo = false;                    // Use PBO for async transfer (disabled due to stability issues)
};

/// @brief RGBD Camera renderer using MuJoCo offscreen rendering
class CameraRenderer {
 public:
  explicit CameraRenderer(const CameraConfig& config);
  ~CameraRenderer();

  // Disable copy
  CameraRenderer(const CameraRenderer&) = delete;
  CameraRenderer& operator=(const CameraRenderer&) = delete;

  /// @brief Initialize the renderer with a MuJoCo model
  /// @param model Pointer to the MuJoCo model
  /// @param existing_context Optional existing mjrContext to share (for OpenGL context)
  /// @return true if initialization succeeded
  bool Initialize(const mjModel* model, mjrContext* existing_context = nullptr);

  /// @brief Render camera images from the current simulation state
  /// @param model Pointer to the MuJoCo model
  /// @param data Pointer to the MuJoCo data
  /// @return true if rendering succeeded
  bool Render(const mjModel* model, const mjData* data);

  /// @brief Get the RGB image buffer (flipped for standard image coordinates)
  /// @return Reference to RGB buffer (width * height * 3 bytes)
  const std::vector<uint8_t>& GetRgbImage() const { return rgb_buffer_; }

  /// @brief Get the depth image buffer (in meters)
  /// @return Reference to depth buffer (width * height floats)
  const std::vector<float>& GetDepthImage() const { return depth_buffer_; }

  /// @brief Get the camera ID in the MuJoCo model
  int GetCameraId() const { return camera_id_; }

  /// @brief Get the camera configuration
  const CameraConfig& GetConfig() const { return config_; }

  /// @brief Check if the renderer is initialized
  bool IsInitialized() const { return initialized_; }

 private:
  CameraConfig config_;
  int camera_id_ = -1;

  // MuJoCo visualization objects
  mjvScene scene_;
  mjvCamera camera_;
  mjvOption option_;
  mjrContext context_;
  bool owns_context_ = false;  // Whether we own the context and need to free it

  // Image buffers
  std::vector<uint8_t> rgb_buffer_;   // RGB: width * height * 3
  std::vector<float> depth_buffer_;   // Depth: width * height

  bool initialized_ = false;

  // === PBO (Pixel Buffer Object) for async GPU->CPU transfer ===
  // Double-buffering: while GPU writes to one PBO, CPU reads from the other
  static constexpr int NUM_PBOS = 2;
  unsigned int pbo_rgb_[NUM_PBOS] = {0, 0};    // PBOs for RGB data
  unsigned int pbo_depth_[NUM_PBOS] = {0, 0};  // PBOs for depth data
  int pbo_index_ = 0;                           // Current PBO index (ping-pong)
  bool pbo_initialized_ = false;
  int frame_count_ = 0;                         // Frame counter for PBO sync

  /// @brief Initialize PBO buffers
  bool InitializePBOs();

  /// @brief Cleanup PBO buffers
  void CleanupPBOs();

  /// @brief Render using PBO async transfer
  bool RenderWithPBO(const mjModel* model, const mjData* data);

  /// @brief Render using synchronous transfer (fallback)
  bool RenderSync(const mjModel* model, const mjData* data);

  /// @brief Flip image vertically (OpenGL origin is bottom-left)
  void FlipImageVertically();
};

}  // namespace mujoco

#endif  // MUJOCO_CAMERA_RENDERER_H_

