// Copyright 2026 EngineAI Robotics
// RGBD Camera Renderer Implementation
// Optimized with PBO (Pixel Buffer Object) for async GPU->CPU transfer

#include "camera_renderer.h"

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

// OpenGL extension function pointers for PBO
// These are loaded dynamically at runtime via glfwGetProcAddress
typedef void (APIENTRY *PFNGLGENBUFFERSPROC)(GLsizei n, GLuint *buffers);
typedef void (APIENTRY *PFNGLDELETEBUFFERSPROC)(GLsizei n, const GLuint *buffers);
typedef void (APIENTRY *PFNGLBINDBUFFERPROC)(GLenum target, GLuint buffer);
typedef void (APIENTRY *PFNGLBUFFERDATAPROC)(GLenum target, GLsizeiptr size, const void *data, GLenum usage);
typedef void* (APIENTRY *PFNGLMAPBUFFERPROC)(GLenum target, GLenum access);
typedef GLboolean (APIENTRY *PFNGLUNMAPBUFFERPROC)(GLenum target);

static PFNGLGENBUFFERSPROC pglGenBuffers = nullptr;
static PFNGLDELETEBUFFERSPROC pglDeleteBuffers = nullptr;
static PFNGLBINDBUFFERPROC pglBindBuffer = nullptr;
static PFNGLBUFFERDATAPROC pglBufferData = nullptr;
static PFNGLMAPBUFFERPROC pglMapBuffer = nullptr;
static PFNGLUNMAPBUFFERPROC pglUnmapBuffer = nullptr;

// OpenGL constants for PBO
#ifndef GL_PIXEL_PACK_BUFFER
#define GL_PIXEL_PACK_BUFFER 0x88EB
#endif
#ifndef GL_STREAM_READ
#define GL_STREAM_READ 0x88E1
#endif
#ifndef GL_READ_ONLY
#define GL_READ_ONLY 0x88B8
#endif

static bool LoadPBOFunctions() {
  pglGenBuffers = (PFNGLGENBUFFERSPROC)glfwGetProcAddress("glGenBuffers");
  pglDeleteBuffers = (PFNGLDELETEBUFFERSPROC)glfwGetProcAddress("glDeleteBuffers");
  pglBindBuffer = (PFNGLBINDBUFFERPROC)glfwGetProcAddress("glBindBuffer");
  pglBufferData = (PFNGLBUFFERDATAPROC)glfwGetProcAddress("glBufferData");
  pglMapBuffer = (PFNGLMAPBUFFERPROC)glfwGetProcAddress("glMapBuffer");
  pglUnmapBuffer = (PFNGLUNMAPBUFFERPROC)glfwGetProcAddress("glUnmapBuffer");
  
  return pglGenBuffers && pglDeleteBuffers && pglBindBuffer && 
         pglBufferData && pglMapBuffer && pglUnmapBuffer;
}

namespace mujoco {

CameraRenderer::CameraRenderer(const CameraConfig& config) : config_(config) {
  // Pre-allocate image buffers
  rgb_buffer_.resize(config_.width * config_.height * 3);
  depth_buffer_.resize(config_.width * config_.height);
}

CameraRenderer::~CameraRenderer() {
  CleanupPBOs();
  if (initialized_ && owns_context_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&context_);
  }
}

bool CameraRenderer::InitializePBOs() {
  if (pbo_initialized_) {
    return true;
  }

  // Load PBO extension functions
  if (!LoadPBOFunctions()) {
    std::cerr << "[CameraRenderer] Warning: PBO functions not available. "
              << "Falling back to synchronous mode." << std::endl;
    return false;
  }

  const size_t rgb_size = config_.width * config_.height * 3;
  const size_t depth_size = config_.width * config_.height * sizeof(float);

  // Generate PBOs for RGB
  pglGenBuffers(NUM_PBOS, pbo_rgb_);
  for (int i = 0; i < NUM_PBOS; ++i) {
    pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_rgb_[i]);
    pglBufferData(GL_PIXEL_PACK_BUFFER, rgb_size, nullptr, GL_STREAM_READ);
  }

  // Generate PBOs for depth
  pglGenBuffers(NUM_PBOS, pbo_depth_);
  for (int i = 0; i < NUM_PBOS; ++i) {
    pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_depth_[i]);
    pglBufferData(GL_PIXEL_PACK_BUFFER, depth_size, nullptr, GL_STREAM_READ);
  }

  // Unbind
  pglBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

  // Check for errors
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    std::cerr << "[CameraRenderer] Warning: PBO initialization failed (OpenGL error: " << err << "). "
              << "Falling back to synchronous mode." << std::endl;
    CleanupPBOs();
    return false;
  }

  pbo_initialized_ = true;
  std::cout << "[CameraRenderer] PBO async transfer enabled (double-buffering)" << std::endl;
  return true;
}

void CameraRenderer::CleanupPBOs() {
  if (pbo_rgb_[0] != 0 && pglDeleteBuffers) {
    pglDeleteBuffers(NUM_PBOS, pbo_rgb_);
    pbo_rgb_[0] = pbo_rgb_[1] = 0;
  }
  if (pbo_depth_[0] != 0 && pglDeleteBuffers) {
    pglDeleteBuffers(NUM_PBOS, pbo_depth_);
    pbo_depth_[0] = pbo_depth_[1] = 0;
  }
  pbo_initialized_ = false;
}

bool CameraRenderer::Initialize(const mjModel* model, mjrContext* existing_context) {
  if (!model) {
    std::cerr << "[CameraRenderer] Error: Invalid model pointer" << std::endl;
    return false;
  }

  // Find camera by name in the model
  camera_id_ = mj_name2id(model, mjOBJ_CAMERA, config_.name.c_str());
  if (camera_id_ < 0) {
    std::cerr << "[CameraRenderer] Error: Camera '" << config_.name << "' not found in model" << std::endl;
    std::cerr << "[CameraRenderer] Available cameras: ";
    for (int i = 0; i < model->ncam; ++i) {
      std::cerr << mj_id2name(model, mjOBJ_CAMERA, i) << " ";
    }
    std::cerr << std::endl;
    return false;
  }

  // Initialize visualization scene
  mjv_defaultScene(&scene_);
  mjv_makeScene(model, &scene_, 2000);

  // Initialize camera view settings
  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = camera_id_;

  // Initialize rendering options
  mjv_defaultOption(&option_);
  // Disable some visual effects for faster rendering
  option_.flags[mjVIS_CONTACTPOINT] = 0;
  option_.flags[mjVIS_CONTACTFORCE] = 0;

  // Use existing rendering context if provided
  if (existing_context) {
    std::memcpy(&context_, existing_context, sizeof(mjrContext));
    owns_context_ = false;
    std::cout << "[CameraRenderer] Using existing context, offscreen: " 
              << context_.offWidth << "x" << context_.offHeight 
              << ", offFBO=" << context_.offFBO << std::endl;
  } else {
    mjr_defaultContext(&context_);
    mjr_makeContext(model, &context_, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &context_);
    
    std::cout << "[CameraRenderer] Created new context, offscreen: " 
              << context_.offWidth << "x" << context_.offHeight 
              << ", offFBO=" << context_.offFBO << std::endl;
    
    if (context_.offFBO == 0) {
      std::cerr << "[CameraRenderer] Warning: Offscreen framebuffer not available." << std::endl;
      mjv_freeScene(&scene_);
      return false;
    }
    owns_context_ = true;
  }
  
  // Check if offscreen buffer is large enough for our requested resolution
  if (context_.offWidth < config_.width || context_.offHeight < config_.height) {
    std::cerr << "[CameraRenderer] Warning: Offscreen buffer (" << context_.offWidth << "x" << context_.offHeight
              << ") is smaller than requested resolution (" << config_.width << "x" << config_.height << ")!" << std::endl;
    std::cerr << "[CameraRenderer] Images will be clipped or corrupted. Consider reducing resolution." << std::endl;
  }

  // Try to initialize PBOs for async transfer
  if (config_.use_pbo) {
    InitializePBOs();
  }

  initialized_ = true;
  std::cout << "[CameraRenderer] Initialized camera '" << config_.name << "' (ID: " << camera_id_ 
            << ") with resolution " << config_.width << "x" << config_.height 
            << (pbo_initialized_ ? " [PBO async]" : " [sync]") << std::endl;
  return true;
}

bool CameraRenderer::Render(const mjModel* model, const mjData* data) {
  if (!initialized_ || !model || !data) {
    return false;
  }

  if (pbo_initialized_ && config_.use_pbo) {
    return RenderWithPBO(model, data);
  } else {
    return RenderSync(model, data);
  }
}

bool CameraRenderer::RenderWithPBO(const mjModel* model, const mjData* data) {
  // PBO double-buffering async transfer:
  // Frame N: 
  //   1. Start async read to PBO[current]
  //   2. Map and read from PBO[previous] (data from frame N-1)
  //
  // This allows GPU rendering and CPU reading to happen in parallel!

  const int current_pbo = pbo_index_;
  const int previous_pbo = (pbo_index_ + 1) % NUM_PBOS;
  
  // Update scene
  mjv_updateScene(model, const_cast<mjData*>(data), &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

  // Set viewport and render
  mjrRect viewport = {0, 0, config_.width, config_.height};
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  mjr_render(viewport, &scene_, &context_);

  const size_t rgb_size = config_.width * config_.height * 3;
  (void)rgb_size;  // Used in memcpy below

  // === Step 1: Start async read into current PBO ===
  // Bind current PBO for RGB and initiate async read
  pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_rgb_[current_pbo]);
  glReadPixels(0, 0, config_.width, config_.height, GL_RGB, GL_UNSIGNED_BYTE, nullptr);

  // Bind current PBO for depth and initiate async read
  pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_depth_[current_pbo]);
  glReadPixels(0, 0, config_.width, config_.height, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);

  // === Step 2: Read from previous PBO (if we have data from previous frame) ===
  if (frame_count_ > 0) {
    // Map previous RGB PBO and copy data
    pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_rgb_[previous_pbo]);
    void* rgb_ptr = pglMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    if (rgb_ptr) {
      std::memcpy(rgb_buffer_.data(), rgb_ptr, config_.width * config_.height * 3);
      pglUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    }

    // Map previous depth PBO and copy data
    pglBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_depth_[previous_pbo]);
    void* depth_ptr = pglMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    if (depth_ptr) {
      // Copy and convert depth values
      const float* raw_depth = static_cast<const float*>(depth_ptr);
      const float extent = model->stat.extent;
      const float znear = model->vis.map.znear * extent;
      const float zfar = model->vis.map.zfar * extent;
      const float zfar_plus_znear = zfar + znear;
      const float zfar_minus_znear = zfar - znear;
      const float two_znear_zfar = 2.0f * znear * zfar;
      const size_t pixel_count = config_.width * config_.height;

      for (size_t i = 0; i < pixel_count; ++i) {
        const float d = raw_depth[i];
        if (d >= 1.0f) {
          depth_buffer_[i] = std::numeric_limits<float>::infinity();
        } else if (d <= 0.0f) {
          depth_buffer_[i] = 0.0f;
        } else {
          const float ndc_depth = 2.0f * d - 1.0f;
          depth_buffer_[i] = two_znear_zfar / (zfar_plus_znear - ndc_depth * zfar_minus_znear);
        }
      }
      pglUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    }

    // Flip images
    FlipImageVertically();
  }

  // Unbind PBO
  pglBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

  // Switch to next PBO for next frame
  pbo_index_ = (pbo_index_ + 1) % NUM_PBOS;
  frame_count_++;

  // First frame has no previous data, return false to skip publishing
  return frame_count_ > 1;
}

bool CameraRenderer::RenderSync(const mjModel* model, const mjData* data) {
  // Original synchronous rendering (fallback)
  mjv_updateScene(model, const_cast<mjData*>(data), &option_, nullptr, &camera_, mjCAT_ALL, &scene_);

  mjrRect viewport = {0, 0, config_.width, config_.height};
  
  // Switch to offscreen buffer for camera rendering
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  mjr_render(viewport, &scene_, &context_);

  // Synchronous pixel read (blocking)
  static std::vector<float> raw_depth;
  if (raw_depth.size() != static_cast<size_t>(config_.width * config_.height)) {
    raw_depth.resize(config_.width * config_.height);
  }
  
  mjr_readPixels(rgb_buffer_.data(), raw_depth.data(), viewport, &context_);

  // Convert depth
  const float extent = model->stat.extent;
  const float znear = model->vis.map.znear * extent;
  const float zfar = model->vis.map.zfar * extent;
  const float zfar_plus_znear = zfar + znear;
  const float zfar_minus_znear = zfar - znear;
  const float two_znear_zfar = 2.0f * znear * zfar;
  const size_t pixel_count = raw_depth.size();

  for (size_t i = 0; i < pixel_count; ++i) {
    const float d = raw_depth[i];
    if (d >= 1.0f) {
      depth_buffer_[i] = std::numeric_limits<float>::infinity();
    } else if (d <= 0.0f) {
      depth_buffer_[i] = 0.0f;
    } else {
      const float ndc_depth = 2.0f * d - 1.0f;
      depth_buffer_[i] = two_znear_zfar / (zfar_plus_znear - ndc_depth * zfar_minus_znear);
    }
  }

  FlipImageVertically();
  
  // IMPORTANT: Restore to window buffer after offscreen rendering
  // This prevents the main window from showing black
  mjr_setBuffer(mjFB_WINDOW, &context_);
  
  return true;
}

void CameraRenderer::FlipImageVertically() {
  const int width = config_.width;
  const int height = config_.height;

  // Flip RGB buffer
  const int rgb_row_size = width * 3;
  std::vector<uint8_t> temp_rgb_row(rgb_row_size);
  for (int y = 0; y < height / 2; ++y) {
    int top_idx = y * rgb_row_size;
    int bottom_idx = (height - 1 - y) * rgb_row_size;
    std::memcpy(temp_rgb_row.data(), &rgb_buffer_[top_idx], rgb_row_size);
    std::memcpy(&rgb_buffer_[top_idx], &rgb_buffer_[bottom_idx], rgb_row_size);
    std::memcpy(&rgb_buffer_[bottom_idx], temp_rgb_row.data(), rgb_row_size);
  }

  // Flip depth buffer
  std::vector<float> temp_depth_row(width);
  for (int y = 0; y < height / 2; ++y) {
    int top_idx = y * width;
    int bottom_idx = (height - 1 - y) * width;
    std::memcpy(temp_depth_row.data(), &depth_buffer_[top_idx], width * sizeof(float));
    std::memcpy(&depth_buffer_[top_idx], &depth_buffer_[bottom_idx], width * sizeof(float));
    std::memcpy(&depth_buffer_[bottom_idx], temp_depth_row.data(), width * sizeof(float));
  }
}

}  // namespace mujoco
