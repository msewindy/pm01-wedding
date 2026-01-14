#include "sim_manager.h"
#include <chrono>
#include <cstring>
#include "simulate/array_safety.h"
#include "simulate/glfw_adapter.h"

namespace mj = mujoco;
namespace mju = mujoco::sample_util;

using namespace std::chrono_literals;

// Constants
// Number of DoF for floating base
const int kDofFloatingBase = 6;
// Number of joints for floating base (quaternion + xyz)
const int kNumFloatingBaseJoints = 7;
// maximum mis-alignment before re-sync
constexpr double kSyncMisalign = 0.1;
// fraction of refresh available for simulation
constexpr double kSimRefreshFraction = 0.7;
// Initialize static member
const std::chrono::milliseconds kBusyWaitTime(1);

// Static wrapper function for MuJoCo control callback
static void TorqueControllerWrapper(const mjModel* m, mjData* d) { SimManager::GetInstance().TorqueController(m, d); }

SimManager& SimManager::GetInstance() {
  static SimManager instance;
  return instance;
}

SimManager::SimManager() { node_ = std::make_shared<rclcpp::Node>("mujoco_simulator"); }

SimManager::~SimManager() {
  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }
  mj_deleteData(d_);
  mj_deleteModel(m_);
  ros_interface_.reset();
}

void SimManager::TorqueController(const mjModel* m, mjData* d) {
  if (!ros_interface_) {
    return;
  }

  // 检查是否收到了第一条有效命令
  bool has_command = ros_interface_->HasReceivedFirstCommand();
  
  // Get thread-safe copy of the command values
  auto cmd = ros_interface_->GetCommandedSafe();

  // Check if this is a floating base robot
  bool is_floating_base = (m->nv != m->nu);

  // Debug log - 每 5000 次打印一次（约每秒1次 @ 仿真5000Hz）
  static int ctrl_count = 0;
  ctrl_count++;
  if (ctrl_count % 5000 == 1) {
    RCLCPP_INFO(node_->get_logger(), 
      "[DEBUG-CTRL] TorqueController: has_cmd=%d, cmd.pos.size=%zu, stiffness[0]=%.1f, is_floating=%d",
      has_command ? 1 : 0,
      cmd.position.size(), 
      cmd.stiffness.size() > 0 ? cmd.stiffness[0] : -1.0,
      is_floating_base ? 1 : 0);
  }

  // Apply commanded controls
  for (int i = 0; i < m->nu; ++i) {
    if (i >= cmd.position.size() || i >= cmd.velocity.size() || i >= cmd.torque.size() ||
        i >= cmd.feed_forward_torque.size() || i >= cmd.stiffness.size() || i >= cmd.damping.size()) {
      continue;
    }

    // Get current position and velocity, accounting for floating base if needed
    double current_position;
    double current_velocity;

    if (is_floating_base) {
      current_position = d->qpos[i + kNumFloatingBaseJoints];
      current_velocity = d->qvel[i + kDofFloatingBase];
    } else {
      current_position = d->qpos[i];
      current_velocity = d->qvel[i];
    }

    // 目标位置：如果还没收到有效命令，使用当前位置保持姿态
    double target_position;
    double stiffness;
    double damping;
    
    if (!has_command) {
      // 还没收到命令，使用当前位置保持姿态（高刚度）
      target_position = current_position;
      stiffness = 400.0;
      damping = 5.0;
    } else {
      // 使用收到的命令
      target_position = cmd.position[i];
      stiffness = cmd.stiffness[i];
      damping = cmd.damping[i];
    }

    // PD control with feed-forward torque
    double position_error = target_position - current_position;
    double velocity_error = cmd.velocity[i] - current_velocity;

    d->ctrl[i] = cmd.feed_forward_torque[i] + stiffness * position_error + damping * velocity_error;
  }
}

bool SimManager::Initialize() {
  auto logger = node_->get_logger();
  if (rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_INFO) != RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to set logger level");
    return false;
  }

  RCLCPP_INFO(logger, "MuJoCo Simulator node initialized");

  // Verify environment variables
  if (!std::getenv("PRODUCT") || !std::getenv("MUJOCO_ASSETS_PATH")) {
    RCLCPP_ERROR(logger, "Required environment variables not set! Please run from launch file.");
    return false;
  }

  // Get product name and resource path from environment variables
  std::string product_name = std::string(std::getenv("PRODUCT"));
  std::string assets_path = std::string(std::getenv("MUJOCO_ASSETS_PATH"));

  // Construct config file path
  std::string config_file = assets_path + "/config/" + product_name + ".yaml";
  RCLCPP_INFO(logger, "Loading config from %s", config_file.c_str());

  // Initialize config loader
  config_loader_ = std::make_shared<ConfigLoader>(config_file);
  config_loader_->SetAssetsPath(assets_path);
  if (!config_loader_->LoadConfig()) {
    RCLCPP_ERROR(logger, "Failed to load config file: %s", config_file.c_str());
    return false;
  }

  // Create the MuJoCo ROS interface
  ros_interface_ = std::make_unique<mujoco::RosInterface>(node_, config_loader_);
  if (!ros_interface_->Initialize()) {
    RCLCPP_ERROR(logger, "Failed to initialize MuJoCo ROS interface");
    return false;
  }

  // Install control callback
  mjcb_control = TorqueControllerWrapper;

  // Log version
  RCLCPP_INFO(logger, "MuJoCo version %s", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    RCLCPP_ERROR(logger, "Headers and library have different versions");
    return false;
  }

  // Setup camera, option, perturb
  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // Create simulation object
  sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, false);

  // Set post-render callback for camera rendering
  sim_->SetPostRenderCallback([this](const mjModel* m, const mjData* d, mjrContext* ctx) {
    this->OnPostRender(m, d, ctx);
  });

  return true;
}

void SimManager::Run() {
  auto logger = node_->get_logger();
  std::string model_file = config_loader_->GetModelFilePath();
  RCLCPP_INFO(logger, "Model file path: %s", model_file.c_str());

  // Set VFS directory before starting physics thread
  const std::string resource_dir = config_loader_->GetResourceDir();
  setenv("MJCF_PATH", resource_dir.c_str(), 1);
  RCLCPP_INFO(logger, "Setting MJCF_PATH environment variable: %s", resource_dir.c_str());

  // Start physics thread
  RCLCPP_INFO(logger, "Starting physics thread");
  physics_thread_ = std::thread([this, model_file]() { PhysicsThread(model_file); });

  // Start UI loop
  RCLCPP_INFO(logger, "Starting UI rendering loop");
  sim_->RenderLoop();
  RCLCPP_INFO(logger, "UI rendering loop completed");
}

mjModel* SimManager::LoadModel(std::string_view file) {
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file.data());
  if (!filename[0]) {
    return nullptr;
  }

  mjModel* mnew = nullptr;
  auto load_start = mj::Simulate::Clock::now();
  auto logger = node_->get_logger();

  if (mju::strlen_arr(filename) > 4 && !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                                                     mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      RCLCPP_ERROR(logger, "Could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, mj_load_error_.data(), mj_load_error_.size());
    if (mj_load_error_[0]) {
      size_t error_length = std::strlen(mj_load_error_.data());
      if (mj_load_error_[error_length - 1] == '\n') {
        mj_load_error_[error_length - 1] = '\0';
      }
      RCLCPP_WARN(logger, "Model compiled with warning: %s", mj_load_error_.data());
      sim_->run = 0;
    }
  }

  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = std::chrono::duration<double>(load_interval).count();

  if (!mnew) {
    RCLCPP_ERROR(logger, "Failed to load model: %s", mj_load_error_.data());
    return nullptr;
  }

  if (load_seconds > 0.25) {
    RCLCPP_INFO(logger, "Model loaded in %.2g seconds", load_seconds);
  }

  try {
    mju::strcpy_arr(sim_->load_error, mj_load_error_.data());
  } catch (...) {
    RCLCPP_ERROR(logger, "Could not copy load error: %s", mj_load_error_.data());
  }

  return mnew;
}

const char* SimManager::Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

void SimManager::HandleDropLoad() {
  sim_->LoadMessage(sim_->dropfilename);
  mjModel* mnew = LoadModel(sim_->dropfilename);
  sim_->droploadrequest.store(false);

  mjData* dnew = nullptr;
  if (mnew) dnew = mj_makeData(mnew);
  if (dnew) {
    sim_->Load(mnew, dnew, sim_->dropfilename);

    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    mj_deleteData(d_);
    mj_deleteModel(m_);

    m_ = mnew;
    d_ = dnew;
    mj_forward(m_, d_);
  } else {
    sim_->LoadMessageClear();
  }
}

void SimManager::HandleUILoad() {
  sim_->uiloadrequest.fetch_sub(1);
  sim_->LoadMessage(sim_->filename);
  mjModel* mnew = LoadModel(sim_->filename);
  mjData* dnew = nullptr;
  if (mnew) dnew = mj_makeData(mnew);
  if (dnew) {
    sim_->Load(mnew, dnew, sim_->filename);

    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    mj_deleteData(d_);
    mj_deleteModel(m_);

    m_ = mnew;
    d_ = dnew;
    mj_forward(m_, d_);
  } else {
    sim_->LoadMessageClear();
  }
}

void SimManager::PhysicsLoop() {
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  while (!sim_->exitrequest.load()) {
    if (ros_interface_) {
      rclcpp::spin_some(ros_interface_->GetNode());
    }

    if (sim_->droploadrequest.load()) {
      HandleDropLoad();
    }

    if (sim_->uiloadrequest.load()) {
      HandleUILoad();
    }

    if (sim_->run && sim_->busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(kBusyWaitTime);
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

      if (m_) {
        if (sim_->run) {
          bool stepped = false;
          const auto startCPU = mj::Simulate::Clock::now();
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d_->time - syncSim;
          double slowdown = 100 / sim_->percentRealTime[sim_->real_time_index];
          bool misaligned = std::abs((elapsedCPU / slowdown).count() - elapsedSim) > kSyncMisalign;

          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim_->speed_changed) {
            syncCPU = startCPU;
            syncSim = d_->time;
            sim_->speed_changed = false;

            ros_interface_->UpdatePhotoPanelPosition();  // Update photo panel before step
            mj_step(m_, d_);
            ros_interface_->UpdateSimState(m_, d_);
            const char* message = Diverged(m_->opt.disableflags, d_);
            if (message) {
              sim_->run = 0;
              mju::strcpy_arr(sim_->load_error, message);
            } else {
              stepped = true;
            }
          } else {
            bool measured = false;
            mjtNum prevSim = d_->time;
            double refreshTime = kSimRefreshFraction / sim_->refresh_rate;

            while ((d_->time - syncSim) * slowdown < (mj::Simulate::Clock::now() - syncCPU).count() &&
                   mj::Simulate::Clock::now() - startCPU < refreshTime * 1s) {
              if (!measured && elapsedSim) {
                sim_->measured_slowdown = elapsedCPU.count() / elapsedSim;
                measured = true;
              }

              ros_interface_->UpdatePhotoPanelPosition();  // Update photo panel before step
              sim_->InjectNoise();
              mj_step(m_, d_);
              ros_interface_->UpdateSimState(m_, d_);
              const char* message = Diverged(m_->opt.disableflags, d_);
              if (message) {
                sim_->run = 0;
                mju::strcpy_arr(sim_->load_error, message);
              } else {
                stepped = true;
              }

              if (d_->time < prevSim) {
                break;
              }
            }
          }

          if (stepped) {
            sim_->AddToHistory();
          }
        } else {
          mj_forward(m_, d_);
          sim_->speed_changed = true;
          // Publish joint state even when simulation is paused
          if (ros_interface_) {
            ros_interface_->UpdateSimState(m_, d_);
          } 
        }
      }
    }
  }
}

void SimManager::PhysicsThread(std::string_view filename) {
  if (!rclcpp::ok()) {
    std::cerr << "ROS context not initialized in physics thread!" << std::endl;
    return;
  }

  auto logger = rclcpp::get_logger("mujoco_physics");
  if (rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_INFO) != RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to set logger level");
  }

  RCLCPP_INFO(logger, "PhysicsThread started, filename: %s", filename.data());

  if (mjcb_control != &TorqueControllerWrapper) {
    RCLCPP_WARN(logger, "Control callback not set correctly, setting mjcb_control now");
    mjcb_control = &TorqueControllerWrapper;
  } else {
    RCLCPP_INFO(logger, "MuJoCo control callback is correctly set");
  }

  if (!filename.empty()) {
    sim_->LoadMessage(filename.data());
    m_ = LoadModel(filename);
    if (m_) {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      d_ = mj_makeData(m_);

      RCLCPP_INFO(logger, "Setting model and data to ROS interface...");
      if (ros_interface_) {
        ros_interface_->SetModelAndData(m_, d_);
        RCLCPP_INFO(logger, "ROS interface successfully set model and data");
        
        // Check if camera exists in model
        int cam_id = mj_name2id(m_, mjOBJ_CAMERA, "head_rgbd_camera");
        if (cam_id >= 0) {
          RCLCPP_INFO(logger, "RGBD camera 'head_rgbd_camera' found in model (ID: %d)", cam_id);
          RCLCPP_INFO(logger, "Camera will be initialized in render thread for ROS2 image publishing");
        } else {
          RCLCPP_WARN(logger, "RGBD camera 'head_rgbd_camera' not found in model");
        }
      } else {
        RCLCPP_ERROR(logger, "Error: ros_interface is null!");
      }
    }
    if (d_) {
      sim_->Load(m_, d_, filename.data());
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

      RCLCPP_INFO(logger, "Performing initial mj_forward calculation...");
      mj_forward(m_, d_);
      RCLCPP_INFO(logger, "mj_forward calculation completed");
      
      // Publish initial joint state immediately after model is loaded
      if (ros_interface_) {
        RCLCPP_INFO(logger, "Publishing initial joint state...");
        ros_interface_->UpdateSimState(m_, d_);
      }
    } else {
      sim_->LoadMessageClear();
    }
  }

  if (ros_interface_) {
    RCLCPP_INFO(logger, "Starting physics loop, control callback status: %s",
                (mjcb_control == &TorqueControllerWrapper ? "set" : "not set"));
  }

  RCLCPP_INFO(logger, "Starting physics loop");
  PhysicsLoop();

  RCLCPP_INFO(logger, "Physics thread ending, cleaning up resources");
}

void SimManager::OnPostRender(const mjModel* m, const mjData* d, mjrContext* context) {
  // This is called from the render thread after each frame
  // 
  // PERFORMANCE NOTE: Set ENABLE_CAMERA_RENDERING to false to disable camera
  // and test MuJoCo's native rendering performance
  constexpr bool ENABLE_CAMERA_RENDERING = true;
  
  if (!ENABLE_CAMERA_RENDERING) {
    return;  // Skip camera rendering entirely for performance testing
  }
  
  // Initialize camera if not already done
  static bool camera_initialized = false;
  static bool camera_init_logged = false;
  
  if (!ros_interface_ || !m || !d || !context) {
    return;
  }

  // Try to initialize camera on first call (now we have valid OpenGL context)
  // Pass the shared context so camera renderer can reuse loaded textures
  // Try to initialize camera on first call (now we have valid OpenGL context)
  // Pass the shared context so camera renderer can reuse loaded textures
  std::lock_guard<std::recursive_mutex> lock(sim_->mtx);
  if (!camera_initialized) {
    if (ros_interface_->InitializeCamera(m, context)) {
      camera_initialized = true;
      if (!camera_init_logged) {
        RCLCPP_INFO(node_->get_logger(), "RGBD camera initialized in render thread (using shared context)");
        camera_init_logged = true;
      }
    }
  }

  // Render and publish camera images
  if (camera_initialized) {
    ros_interface_->RenderAndPublishCamera(m, d);
  }
}