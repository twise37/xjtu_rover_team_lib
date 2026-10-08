#include "csi_capture.hpp"
#include "localization.hpp"
#include "auv_core/semantic_map.hpp"
#include "auv_core/status_decoder.hpp"
#include "auv_control/route_executor.hpp"
#include "auv_mapping/grid_mapper.hpp"
#include "auv_mission/mission_fsm.hpp"
#include "auv_planning/grid_planner.hpp"
#include "auv_stm32_bridge/motion_target.hpp"
#include "auv_stm32_bridge/gripper_protocol.hpp"
#include "auv_stm32_bridge/protocol.h"
#include "auv_stm32_bridge/serial_port.hpp"
#include "auv_stm32_bridge/stream_parser.hpp"
#include "auv_vision/apriltag_detector.hpp"
#include "auv_vision/camera_source.hpp"
#include "auv_vision/cone_detector.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <memory>
#include <sstream>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/wait.h>
#ifdef AUV_HAVE_HTTPLIB
#include <httplib.h>
#endif

using Clock = std::chrono::steady_clock;
static double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
static std::atomic<bool> running{true};
static void stop_signal(int) { running = false; }
static std::string json_escape(const std::string& s) {
  std::string out;
  for (char c : s) { if (c == '"' || c == '\\') out += '\\'; if (c >= 32) out += c; }
  return out;
}
static bool propulsion_phase(auv_mission::MissionPhase phase) {
  switch (phase) {
    case auv_mission::MissionPhase::kVisitCones:
    case auv_mission::MissionPhase::kSearchCucumber:
    case auv_mission::MissionPhase::kAlignCucumber:
    case auv_mission::MissionPhase::kGrab:
    case auv_mission::MissionPhase::kTransport:
    case auv_mission::MissionPhase::kRelease:
    case auv_mission::MissionPhase::kSearchValve:
    case auv_mission::MissionPhase::kAlignValve:
    case auv_mission::MissionPhase::kRotateValve:
    case auv_mission::MissionPhase::kReturnHome:
    case auv_mission::MissionPhase::kSurface:
      return true;
    default:
      return false;
  }
}
struct Config {
  Localization::Settings localization;
  std::string camera, serial, socket, log, debug_dir;
  bool front_enabled{};
  std::string front_source{"/dev/v4l/by-id/REPLACE_WITH_FRONT_USB_CAMERA"};
  int front_width{320},front_height{240},front_fps{30};
  std::string operation_mode{"debug"};
  bool auto_start{}, auto_arm{};
  double startup_delay{5.0}, startup_timeout{30.0}, startup_stable{1.0};
  int camera_width{640}, camera_height{480};
  double camera_fps{30.0};
  std::string camera_pixel_format{"MJPG"}, apriltag_family{"tag36h11"};
  int baud{}, expected_cones{4};
  double status_timeout{0.5}, frame_timeout{0.5}, pose_timeout{0.5}, control_watchdog_timeout{0.25};
  double actuator_status_timeout{0.5};
  bool motion_enabled{}, directions_calibrated{}, limits_calibrated{};
  bool surface_before_traversal{};
  double surface_depth_m{0.1}, surface_tolerance_m{0.05}, surface_stable_sec{1.0};
  double surface_timeout_sec{30.0};
  double depth_deadzone_m{0.05}, depth_recover_stable_sec{0.5}, depth_pause_timeout_sec{10.0};
  double depart_speed{0.08}, depart_duration_sec{2.0};
  auv_control::RouteExecutorConfig route;
  auv_mapping::GridMapperConfig grid;
  auv_vision::ConeDetectorConfig cone;
  auv_vision::ConeTrackerConfig tracker;
  auv_mission::MissionFsmConfig mission;
  std::string mission_profile{"task_one"};
  auv_planning::GridPlannerConfig planner;
  bool video_enabled{}, software_fallback{};
  std::string video_dir,video_encoder,web_bind,web_assets;
  int web_port{},video_width{640},video_height{480},video_fps{20},video_bitrate_kbps{2000};
  double segment_time{0.5};
  std::uint64_t max_event_bytes{10*1024*1024};
  std::vector<double> camera_matrix, distortion;
};
static Config load_config(const std::string& path) {
  const auto y = YAML::LoadFile(path);
  Config c;
  c.localization=Localization::config(y["localization"]);
  c.camera = y["camera"]["source"].as<std::string>();
  c.camera_width = y["camera"]["width"].as<int>();
  c.camera_height = y["camera"]["height"].as<int>();
  c.camera_fps = y["camera"]["fps"].as<double>();
  c.camera_pixel_format = y["camera"]["pixel_format"].as<std::string>();
  if (const auto front=y["camera_front"]) {
    c.front_enabled=front["enabled"].as<bool>(false);
    c.front_source=front["source"].as<std::string>("/dev/v4l/by-id/REPLACE_WITH_FRONT_USB_CAMERA");
    c.front_width=front["width"].as<int>(320);
    c.front_height=front["height"].as<int>(240);
    c.front_fps=front["fps"].as<int>(30);
  }
  if (c.front_enabled && (c.front_width<16 || c.front_width>1920 ||
      c.front_height<16 || c.front_height>1080 || c.front_fps<1 || c.front_fps>60 ||
      (c.front_source!="csi:0" && c.front_source!="csi:1" &&
       c.front_source.rfind("file:",0)!=0 && c.front_source.rfind("/dev/v4l/by-id/",0)!=0) ||
      c.front_source==c.camera))
    throw std::runtime_error("invalid or duplicate front camera configuration");
  c.serial = y["serial"]["device"].as<std::string>();
  c.baud = y["serial"]["baud"].as<int>();
  c.socket = y["control"]["socket"].as<std::string>();
  c.operation_mode = y["operation"]["mode"].as<std::string>();
  c.auto_start = y["operation"]["auto_start"].as<bool>();
  c.auto_arm = y["operation"]["auto_arm"].as<bool>();
  c.startup_delay = y["operation"]["startup_delay_sec"].as<double>();
  c.startup_timeout = y["operation"]["startup_timeout_sec"].as<double>();
  c.startup_stable = y["operation"]["startup_stable_sec"].as<double>();
  c.log = y["logging"]["events"].as<std::string>();
  c.debug_dir = y["logging"]["debug_dir"].as<std::string>();
  c.expected_cones = y["vision"]["expected_cones"].as<int>();
  c.apriltag_family = y["vision"]["apriltag_family"].as<std::string>();
  c.grid.stable_frames = y["vision"]["grid_stable_frames"].as<int>();
  c.cone.minimum_confidence = y["vision"]["cone_minimum_confidence"].as<double>();
  c.tracker.history_size = y["vision"]["cone_history_size"].as<int>();
  c.tracker.required_votes = y["vision"]["cone_required_votes"].as<int>();
  c.tracker.clear_votes = y["vision"]["cone_clear_votes"].as<int>();
  c.planner.maximum_targets = y["planning"]["maximum_targets"].as<std::size_t>();
  c.planner.target_object_types = y["planning"]["target_object_types"].as<std::vector<std::string>>();
  c.planner.blocked_object_types = y["planning"]["blocked_object_types"].as<std::vector<std::string>>();
  c.status_timeout = y["safety"]["status_timeout_sec"].as<double>();
  c.frame_timeout = y["safety"]["frame_timeout_sec"].as<double>();
  c.pose_timeout = y["safety"]["pose_timeout_sec"].as<double>();
  c.control_watchdog_timeout = y["safety"]["control_watchdog_timeout_sec"].as<double>();
  c.actuator_status_timeout = y["safety"]["actuator_status_timeout_sec"].as<double>();
  c.motion_enabled = y["motion"]["motion_commands_enabled"].as<bool>();
  c.directions_calibrated = y["motion"]["directions_calibrated"].as<bool>();
  c.limits_calibrated = y["motion"]["limits_calibrated"].as<bool>();
  c.surface_before_traversal = y["motion"]["surface_before_traversal"].as<bool>(false);
  c.surface_depth_m = y["motion"]["surface_depth_m"].as<double>(0.5);
  c.surface_tolerance_m = y["motion"]["surface_tolerance_m"].as<double>(0.05);
  c.surface_stable_sec = y["motion"]["surface_stable_sec"].as<double>(1.0);
  c.surface_timeout_sec = y["motion"]["surface_timeout_sec"].as<double>(30.0);
  c.depth_deadzone_m = y["motion"]["depth_deadzone_m"].as<double>(0.05);
  c.depth_recover_stable_sec = y["motion"]["depth_recover_stable_sec"].as<double>(0.5);
  c.depth_pause_timeout_sec = y["motion"]["depth_pause_timeout_sec"].as<double>(10.0);
  c.depart_speed = y["motion"]["depart_speed"].as<double>(0.08);
  c.depart_duration_sec = y["motion"]["depart_duration_sec"].as<double>(2.0);
  c.route.surge_from_row = y["motion"]["surge_from_row"].as<double>();
  c.route.surge_from_col = y["motion"]["surge_from_col"].as<double>();
  c.route.sway_from_row = y["motion"]["sway_from_row"].as<double>();
  c.route.sway_from_col = y["motion"]["sway_from_col"].as<double>();
  c.route.maximum_speed = y["motion"]["maximum_speed"].as<double>();
  c.route.arrival_tolerance = y["motion"]["arrival_tolerance"].as<double>();
  c.route.arrival_stable_ticks = y["motion"]["arrival_stable_ticks"].as<int>();
  c.mission.self_check_timeout_sec = y["mission"]["self_check_timeout_sec"].as<double>();
  c.mission.apriltag_timeout_sec = y["mission"]["apriltag_timeout_sec"].as<double>();
  c.mission.map_timeout_sec = y["mission"]["map_timeout_sec"].as<double>();
  c.mission.planning_timeout_sec = y["mission"]["planning_timeout_sec"].as<double>();
  c.mission.cone_visit_timeout_sec = y["mission"]["cone_visit_timeout_sec"].as<double>();
  c.mission_profile = y["mission"]["profile"].as<std::string>();
  c.mission.full_mission = c.mission_profile == "full";
  c.mission.cucumber_search_timeout_sec = y["mission"]["cucumber_search_timeout_sec"].as<double>();
  c.mission.cucumber_align_timeout_sec = y["mission"]["cucumber_align_timeout_sec"].as<double>();
  c.mission.gripper_timeout_sec = y["mission"]["gripper_timeout_sec"].as<double>();
  c.mission.transport_timeout_sec = y["mission"]["transport_timeout_sec"].as<double>();
  c.mission.valve_search_timeout_sec = y["mission"]["valve_search_timeout_sec"].as<double>();
  c.mission.valve_align_timeout_sec = y["mission"]["valve_align_timeout_sec"].as<double>();
  c.mission.valve_rotate_timeout_sec = y["mission"]["valve_rotate_timeout_sec"].as<double>();
  c.mission.return_home_timeout_sec = y["mission"]["return_home_timeout_sec"].as<double>();
  c.mission.surface_timeout_sec = y["mission"]["surface_timeout_sec"].as<double>();
  c.mission.status_timeout_sec = c.status_timeout;
  c.mission.allow_armed_during_visit = true;
  c.video_enabled = y["video"]["enabled"].as<bool>();
  c.video_dir = y["video"]["directory"].as<std::string>();
  c.video_encoder = y["video"]["encoder"].as<std::string>();
  c.video_width = y["video"]["width"].as<int>();
  c.video_height = y["video"]["height"].as<int>();
  c.video_fps = y["video"]["fps"].as<int>();
  c.video_bitrate_kbps = y["video"]["bitrate_kbps"].as<int>();
  c.segment_time = y["video"]["segment_time_sec"].as<double>();
  c.software_fallback = y["video"]["software_fallback_enabled"].as<bool>();
  c.web_bind = y["web"]["bind"].as<std::string>();
  c.web_port = y["web"]["port"].as<int>();
  c.web_assets = y["web"]["assets"].as<std::string>();
  c.max_event_bytes = y["logging"]["max_event_bytes"].as<std::uint64_t>();
  if (y["camera"]["camera_matrix"]) c.camera_matrix = y["camera"]["camera_matrix"].as<std::vector<double>>();
  if (y["camera"]["distortion_coefficients"]) c.distortion = y["camera"]["distortion_coefficients"].as<std::vector<double>>();
  if (c.camera != "csi:0" && c.camera != "csi:1" &&
      c.camera.rfind("/dev/v4l/by-id/", 0) != 0 && c.camera.rfind("file:", 0) != 0)
    throw std::runtime_error("camera source must be csi:0/1, /dev/v4l/by-id/... or file:...");
  if (c.camera_width <= 0 || c.camera_width > 1920 || c.camera_height <= 0 ||
      c.camera_height > 1080 || c.camera_fps <= 0 || c.camera_fps > 120 ||
      c.camera_pixel_format.size() != 4 || c.grid.stable_frames <= 0 ||
      c.cone.minimum_confidence <= 0 || c.cone.minimum_confidence > 1 ||
      c.baud <= 0 || c.expected_cones < 1 || c.expected_cones > 9 || c.status_timeout <= 0 ||
      c.frame_timeout <= 0 || c.pose_timeout <= 0 || c.control_watchdog_timeout <= 0 ||
      c.actuator_status_timeout <= 0 ||
      c.control_watchdog_timeout >= c.status_timeout || c.socket.empty() ||
      c.log.empty() || c.debug_dir.empty() || c.startup_delay < 0 ||
      c.startup_timeout <= c.startup_delay || c.startup_stable <= 0 ||
      c.startup_stable >= c.startup_timeout-c.startup_delay)
    throw std::runtime_error("invalid runtime configuration");
  if (c.operation_mode != "debug" && c.operation_mode != "autonomous")
    throw std::runtime_error("operation.mode must be debug or autonomous");
  if (c.mission_profile != "task_one" && c.mission_profile != "full")
    throw std::runtime_error("mission.profile must be task_one or full");
  if (c.operation_mode == "debug" && (c.auto_start || c.auto_arm))
    throw std::runtime_error("automatic operation is only valid in autonomous mode");
  if (c.operation_mode == "autonomous" && !c.auto_start)
    throw std::runtime_error("autonomous mode requires auto_start");
  if (c.auto_arm && !c.motion_enabled)
    throw std::runtime_error("auto_arm requires motion_commands_enabled");
  in_addr web_addr{};
  const bool web_ip_valid = ::inet_pton(AF_INET,c.web_bind.c_str(),&web_addr) == 1;
  const auto web_ip = ntohl(web_addr.s_addr);
  const bool wired_web_address = web_ip_valid &&
    ((web_ip & 0xffffff00U) == 0xc0a88900U ||
     (web_ip & 0xffffff00U) == 0xc0a83200U) &&
    (web_ip & 0xffU) >= 2U && (web_ip & 0xffU) <= 254U;
  if (!wired_web_address || c.web_port <= 0 || c.web_port > 65535 ||
      c.video_dir.empty() || c.web_assets.empty() ||
      c.video_width <= 0 || c.video_width > 1920 ||
      c.video_height <= 0 || c.video_height > 1080 ||
      c.video_fps <= 0 || c.video_fps > 60 ||
      c.video_bitrate_kbps <= 0 || c.segment_time <= 0 || c.max_event_bytes < 1024 ||
      (c.video_encoder != "h264_v4l2m2m" && c.video_encoder != "libx264"))
    throw std::runtime_error("invalid video or wired web configuration");
  if (c.motion_enabled && (c.serial.empty() || !c.directions_calibrated || !c.limits_calibrated ||
      c.route.maximum_speed <= 0 || c.route.maximum_speed > 0.2 ||
      c.camera_matrix.size() != 9 || c.distortion.empty() ||
      !std::isfinite(c.route.surge_from_row*c.route.sway_from_col-
        c.route.surge_from_col*c.route.sway_from_row) ||
      std::abs(c.route.surge_from_row*c.route.sway_from_col-
        c.route.surge_from_col*c.route.sway_from_row) < 1e-6))
    throw std::runtime_error("motion requires calibrated directions, limits and serial device");
  if (c.surface_depth_m < 0.0 || c.surface_tolerance_m <= 0.0 ||
      c.surface_stable_sec <= 0.0 || c.surface_timeout_sec <= 0.0)
    throw std::runtime_error("surface configuration requires non-negative depth and positive tolerance/stable/timeout");
  if (c.depth_deadzone_m < 0.0 || c.depth_recover_stable_sec <= 0.0 ||
      c.depth_pause_timeout_sec <= 0.0 || c.depart_speed <= 0.0 ||
      c.depart_duration_sec <= 0.0)
    throw std::runtime_error("depth watchdog / departure requires non-negative deadzone and positive stable/timeout/speed");
  if (!c.camera_matrix.empty() && c.camera_matrix.size() != 9)
    throw std::runtime_error("camera_matrix must contain 9 values");
  if (!c.distortion.empty() && c.distortion.size() != 4 && c.distortion.size() != 5 &&
      c.distortion.size() != 8 && c.distortion.size() != 12 && c.distortion.size() != 14)
    throw std::runtime_error("distortion_coefficients has invalid size");
  for (auto value : c.camera_matrix) if (!std::isfinite(value)) throw std::runtime_error("camera_matrix contains NaN or infinity");
  for (auto value : c.distortion) if (!std::isfinite(value)) throw std::runtime_error("distortion contains NaN or infinity");
  if (!c.camera_matrix.empty() && (c.camera_matrix[0] <= 0 || c.camera_matrix[4] <= 0))
    throw std::runtime_error("camera focal lengths must be positive");
  return c;
}
class Runtime {
 public:
  explicit Runtime(Config cfg) : cfg_(std::move(cfg)), mission_(cfg_.mission),
    route_(cfg_.route), planner_(cfg_.planner) {
    surface_enabled_ = cfg_.surface_before_traversal;
    (void)auv_vision::AprilTagDetector(cfg_.apriltag_family);
    (void)auv_mapping::GridMapper(cfg_.grid);
    (void)auv_vision::ConeDetector(cfg_.cone);
    (void)auv_vision::ConeTracker(cfg_.tracker);
    std::filesystem::create_directories(std::filesystem::path(cfg_.log).parent_path());
    log_.open(cfg_.log, std::ios::app);
    if (!log_) throw std::runtime_error("cannot open event log");
  }
  void run() {
    std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal); std::signal(SIGPIPE,SIG_IGN);
    event("BOOT", "mode=" + cfg_.operation_mode + " DISARM");
    std::thread logger(&Runtime::logger_loop, this);
    std::thread capture(&Runtime::capture_loop, this);
    std::thread front(&Runtime::front_loop, this);
    std::thread vision(&Runtime::vision_loop, this);
    localization_=std::make_unique<Localization>(cfg_.localization);
    std::thread localization(&Runtime::localization_loop,this);
    std::thread serial(&Runtime::serial_loop, this);
    std::thread control([this] {
      try { control_loop(); }
      catch (const std::exception& e) { fault(std::string("control: ")+e.what()); running=false; }
    });
    std::thread video([this] {
      try { video_loop(); }
      catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        video_detail_=e.what();
      }
    });
#ifdef AUV_HAVE_HTTPLIB
    std::thread web([this] {
      try { web_loop(); }
      catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        web_detail_=e.what();
      }
      web_finished_=true;
    });
#else
    { std::lock_guard<std::mutex> lock(state_mutex_); web_detail_ = "cpp-httplib unavailable"; }
#endif
    try { socket_loop(); }
    catch (const std::exception& e) { fault(std::string("control socket: ")+e.what()); }
    running = false;
#ifdef AUV_HAVE_HTTPLIB
    while (!web_finished_ && !web_server_) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (auto* server=web_server_.load()) server->stop();
    web.join();
#endif
    capture.join(); front.join(); vision.join(); localization.join(); control.join(); serial.join(); video.join();
    event("SHUTDOWN", "DISARM requested");
    log_stop_=true;
    log_cv_.notify_all(); logger.join();
  }
 private:
  enum class TraverseStage { kSurfacing, kTraversing, kDeparting };
  void event(const std::string& name, const std::string& detail) {
    try {
    std::ostringstream line;
    const auto unix_ms=std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    line << "{\"timestamp_unix_ms\":" << unix_ms << ",\"steady_sec\":" << seconds()
         << ",\"event\":\"" << json_escape(name)
         << "\",\"detail\":\"" << json_escape(detail) << "\"}\n";
    { std::lock_guard<std::mutex> lock(log_mutex_);
      if (log_queue_.size() >= 256) { log_queue_.pop_front(); log_degraded_=true; }
      log_queue_.push_back(line.str()); }
    log_cv_.notify_one();
    } catch (...) { log_degraded_=true; }
  }
  void logger_loop() {
    while (true) {
      std::string line;
      { std::unique_lock<std::mutex> lock(log_mutex_);
        log_cv_.wait_for(lock,std::chrono::milliseconds(100),[this]{return log_stop_ || !log_queue_.empty();});
        if (log_queue_.empty() && log_stop_) break;
        if (!log_queue_.empty()) { line=std::move(log_queue_.front()); log_queue_.pop_front(); } }
      if (line.empty()) continue;
      try {
        if (log_.tellp() >= static_cast<std::streampos>(cfg_.max_event_bytes) ||
            seconds()-log_started_ > 86400) {
          log_.close();
          std::filesystem::rename(cfg_.log,cfg_.log+".1");
          log_.open(cfg_.log,std::ios::trunc);
          log_started_=seconds();
        }
        log_ << line;
        log_.flush();
        if (!log_) log_degraded_=true;
      } catch (...) { log_degraded_=true; }
    }
  }
  void fault(const std::string& why) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    set_fault_locked(why);
    armed_requested_ = false;
    disarm_pending_ = true;
    motion_ = {};
  }
  void set_fault_locked(const std::string& why) {
    if (fault_.empty()) { fault_ = why; event("FAULT",why); }
  }
  bool status_fresh(double now) const { return status_time_ > 0 && now - status_time_ <= cfg_.status_timeout; }
  bool safe_status(double now) const {
    return serial_connected_ && status_fresh(now) &&
      status_.error_flags == 0 && status_.telemetry_valid;
  }
  bool gripper_status_fresh(double now) const {
    return gripper_time_ > 0 && now-gripper_time_ <= cfg_.actuator_status_timeout;
  }
  bool arm_gate_ready(double now) const {
    return cfg_.motion_enabled && cfg_.directions_calibrated && cfg_.limits_calibrated &&
      !cfg_.serial.empty() && fault_.empty() && safe_status(now) && !status_.armed &&
      pose_valid_ && now-pose_time_ <= cfg_.pose_timeout &&
      frame_time_ > 0 && now-frame_time_ <= cfg_.frame_timeout && map_.complete && route_ready_ &&
      mission_.snapshot().phase == auv_mission::MissionPhase::kVisitCones;
  }
  void request_arm_locked(const std::string& source) {
    hold_depth_=status_.depth; hold_yaw_=status_.yaw;
    traverse_stage_ = TraverseStage::kSurfacing;
    surfacing_started_ = 0; surfacing_since_ = 0;
    departing_started_ = 0; departure_done_ = false;
    depth_paused_ = false; depth_paused_since_ = 0; depth_in_band_since_ = 0;
    armed_requested_ = true; arm_ack_ = false; arm_pending_ = true; arm_time_=0;
    event("ARM_REQUEST", source);
  }
  // Precompute the blind departure motion (leave the 3x3 grid after every cone
  // is visited). The exit direction is a unit grid step from the last visited
  // cone: edge/corner cells leave via the nearest outward step, the center cell
  // turns toward an edge midpoint with no cone. The grid step is mapped to body
  // surge/sway through the calibrated route gains, then normalized to
  // depart_speed so departure stays within the calibrated speed envelope.
  void compute_departure() {
    exit_dr_ = 0; exit_dc_ = 0; depart_surge_ = 0.0F; depart_sway_ = 0.0F;
    if (plan_.targets.empty()) return;
    const int r = static_cast<int>(plan_.targets.back().row);
    const int c = static_cast<int>(plan_.targets.back().col);
    auto has_cone = [this](int rr, int cc) {
      for (const auto& t : plan_.targets)
        if (static_cast<int>(t.row) == rr && static_cast<int>(t.col) == cc) return true;
      return false;
    };
    if ((r == 0 || r == 2) && (c == 0 || c == 2)) { exit_dr_ = (r == 0 ? -1 : 1); exit_dc_ = 0; }
    else if (r == 0) { exit_dr_ = -1; exit_dc_ = 0; }
    else if (r == 2) { exit_dr_ = 1; exit_dc_ = 0; }
    else if (c == 0) { exit_dr_ = 0; exit_dc_ = -1; }
    else if (c == 2) { exit_dr_ = 0; exit_dc_ = 1; }
    else {
      const int dirs[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};
      for (const auto& d : dirs) {
        if (!has_cone(1 + d[0], 1 + d[1])) { exit_dr_ = d[0]; exit_dc_ = d[1]; break; }
      }
      if (exit_dr_ == 0 && exit_dc_ == 0) { exit_dr_ = -1; exit_dc_ = 0; }
    }
    double sx = cfg_.route.surge_from_row * exit_dr_ + cfg_.route.surge_from_col * exit_dc_;
    double sy = cfg_.route.sway_from_row  * exit_dr_ + cfg_.route.sway_from_col  * exit_dc_;
    const double mag = std::hypot(sx, sy);
    if (mag > 1e-9) { sx = sx / mag * cfg_.depart_speed; sy = sy / mag * cfg_.depart_speed; }
    depart_surge_ = static_cast<float>(sx);
    depart_sway_ = static_cast<float>(sy);
  }
  bool claim_autonomous_run_locked() {
    const auto latch = cfg_.socket + ".autonomous-started";
    const int fd = ::open(latch.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0640);
    if (fd >= 0) {
      const auto stamp = std::to_string(static_cast<std::uint64_t>(seconds()*1000));
      const auto ignored = ::write(fd,stamp.data(),stamp.size());
      (void)ignored;
      ::close(fd);
      event("AUTONOMOUS_LATCH",latch);
      return true;
    }
    if (errno == EEXIST) set_fault_locked("autonomous mission already started since boot");
    else set_fault_locked("cannot create autonomous start latch");
    disarm_pending_ = true;
    return false;
  }
  void capture_loop() {
    const bool csi=cfg_.camera.rfind("csi:",0)==0;
    auto source_name = cfg_.camera.rfind("file:",0)==0 ? cfg_.camera.substr(5) : cfg_.camera;
    CsiCapture capture;
    auv_vision::CameraSource camera({source_name,cfg_.camera_width,cfg_.camera_height,
      cfg_.camera_fps,cfg_.camera_pixel_format,cfg_.camera.rfind("file:",0)==0});
    double last_ready=seconds();
    bool received=false;
    while (running) {
      try {
        if(csi && !capture.is_open()) {
          capture.open(cfg_.camera.back()-'0',cfg_.camera_width,cfg_.camera_height,
                       static_cast<int>(cfg_.camera_fps));
          last_ready=seconds(); received=false;
        }
        if(!csi && !camera.is_open() && !camera.open()) {
          std::this_thread::sleep_for(std::chrono::seconds(1)); continue;
        }
        cv::Mat image;
        std::vector<std::uint8_t> jpeg;
        const bool ready=csi ? capture.read(image,&jpeg) : camera.read(image);
        if(!ready) {
          if(csi && seconds()-last_ready>(received ? 2.0 : 8.0))
            throw std::runtime_error("down CSI frame timeout");
          if(!csi)camera.close();
          std::this_thread::sleep_for(std::chrono::milliseconds(csi ? 10 : 200));
          continue;
        }
        last_ready=seconds(); received=true;
        const double previous=frame_time_.load();
        if (previous>0 && last_ready>previous)
          down_hz_=down_hz_<=0 ? 1.0/(last_ready-previous) : .9*down_hz_.load()+.1/(last_ready-previous);
        if (!csi && !cv::imencode(".jpg",image,jpeg,{cv::IMWRITE_JPEG_QUALITY,70}))
          throw std::runtime_error("down JPEG encode failed");
        { std::lock_guard<std::mutex> lock(frame_mutex_);
          down_jpeg_=std::move(jpeg);
          frame_=image; frame_time_=last_ready; ++frame_sequence_; ++down_capture_frames_;
        }
      } catch(const std::exception& e) {
        capture.close(); camera.close(); fault(std::string("down camera: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  void front_loop() {
    if (!cfg_.front_enabled) return;
    const bool csi=cfg_.front_source.rfind("csi:",0)==0;
    const auto source=csi ? cfg_.front_source :
      (cfg_.front_source.rfind("file:",0)==0 ? cfg_.front_source.substr(5) : cfg_.front_source);
    CsiCapture capture;
    double last_ready=seconds();
    bool received=false;
    auv_vision::CameraSource camera({source,cfg_.front_width,cfg_.front_height,
      static_cast<double>(cfg_.front_fps),"MJPG",cfg_.front_source.rfind("file:",0)==0});
    while (running) {
      try {
        if (csi && !capture.is_open()) {
          capture.open(source.back()-'0',cfg_.front_width,cfg_.front_height,cfg_.front_fps);
          last_ready=seconds();
          received=false;
        }
        if (!csi && !camera.is_open() && !camera.open()) throw std::runtime_error("front camera unavailable");
        cv::Mat image;
        std::vector<std::uint8_t> jpeg;
        const bool ready=csi ? capture.read(image,&jpeg) : camera.read(image);
        if (!ready) {
          if (csi && seconds()-last_ready>(received ? 2.0 : 8.0)) throw std::runtime_error("CSI frame timeout");
          std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue;
        }
        const double stamp=seconds(), previous=front_time_.load();
        last_ready=stamp;
        received=true;
        if (!csi && !cv::imencode(".jpg",image,jpeg,{cv::IMWRITE_JPEG_QUALITY,70}))
          throw std::runtime_error("front JPEG encode failed");
        { std::lock_guard<std::mutex> lock(front_mutex_);
          front_frame_=image; front_jpeg_=std::move(jpeg); front_time_=stamp; }
        { std::lock_guard<std::mutex> lock(state_mutex_);
          ++front_frames_; front_detail_.clear();
          if (previous>0 && stamp>previous) front_hz_=front_hz_<=0 ? 1.0/(stamp-previous) :
            0.9*front_hz_+0.1/(stamp-previous);
        }
        /* Live V4L2 read already waits for the next frame. Only pace file replay. */
        if (!csi && !camera.is_live())
          std::this_thread::sleep_for(std::chrono::milliseconds(1000/cfg_.front_fps));
      } catch (const std::exception& e) {
        capture.close(); camera.close();
        { std::lock_guard<std::mutex> lock(state_mutex_); front_detail_=e.what(); }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  void localization_loop() {
    std::uint64_t seen=0; double last=0;
    while(running){
      const double now=seconds();
      if(now-last < localization_->period()) {std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}
      last=now;cv::Mat image;double stamp=0;
      {std::lock_guard<std::mutex> lock(frame_mutex_);
       if(seen!=frame_sequence_){seen=frame_sequence_;image=frame_;stamp=frame_time_;}}
      try {
        if(!image.empty()){
          YAML::Node sensor;
          if(!cfg_.serial.empty()){
            std::lock_guard<std::mutex> lock(state_mutex_);
            sensor["stamp"]=status_time_;sensor["valid"]=status_.telemetry_valid;
            sensor["armed"]=status_.armed;sensor["depth_m"]=status_.depth;
            const double degrees=180/std::acos(-1);
            sensor["roll_deg"]=status_.roll*degrees;sensor["pitch_deg"]=status_.pitch*degrees;sensor["yaw_deg"]=status_.yaw*degrees;
          }
          localization_->process(image,stamp,now,sensor);
        }
        else if(!frame_time_ || now-frame_time_>.3)localization_->unavailable("image_unavailable");
        else continue;
        if(localization_->enabled())event("LOCALIZATION",localization_->json(seconds()));
      }catch(const std::exception&){localization_->unavailable("processing_error");}
    }
  }
  void vision_loop() {
    auv_vision::AprilTagDetector tags(cfg_.apriltag_family);
    auv_mapping::GridMapper mapper(cfg_.grid);
    auv_vision::ConeDetector detector(cfg_.cone);
    auv_vision::ConeTracker tracker(cfg_.tracker);
    cv::Mat intrinsics, distortion;
    if (!cfg_.camera_matrix.empty()) {
      intrinsics=cv::Mat(3,3,CV_64F,cfg_.camera_matrix.data()).clone();
      distortion=cv::Mat(cfg_.distortion).clone().reshape(1,1);
    }
    std::uint64_t seen = 0;
    while (running) {
      cv::Mat image; double stamp = 0;
      { std::lock_guard<std::mutex> lock(frame_mutex_);
        if (seen != frame_sequence_) { seen = frame_sequence_; image = frame_; stamp = frame_time_; } }
      if (image.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
      try {
      if (!intrinsics.empty()) {
        cv::Mat corrected;
        cv::undistort(image,corrected,intrinsics,distortion);
        image=std::move(corrected);
      }
      const auto found = tags.detect(image);
      const auto grid = mapper.process(image);
      std::vector<auv_vision::ConeObservation> cones;
      if (grid.stable && !grid.rectified.empty()) cones = tracker.update(detector.process(grid.rectified).observations);
      std::array<bool,9> visited;
      { std::lock_guard<std::mutex> lock(state_mutex_); visited = visited_; }
      const auto map = auv_core::fuse_semantic_map(grid, cones, tracker.ready(), visited, cfg_.expected_cones);
      if (map.complete && !map_image_saved_.exchange(true)) {
        try {
          std::filesystem::create_directories(cfg_.debug_dir);
          const auto filename=cfg_.debug_dir+"/map_"+std::to_string(static_cast<std::uint64_t>(seconds()*1000))+".jpg";
          if (!cv::imwrite(filename,grid.debug_image.empty() ? image : grid.debug_image))
            log_degraded_=true;
          else event("MAP_IMAGE",filename);
        } catch (...) { log_degraded_=true; }
      }
      {
        std::lock_guard<std::mutex> lock(video_mutex_);
        video_frame_ = grid.debug_image.empty() ? image : grid.debug_image;
        ++video_sequence_;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      tag_found_ = tag_found_ || !found.empty();
      map_ = map;
      pose_valid_ = grid.stable && grid.position_valid && std::isfinite(grid.camera_row) && std::isfinite(grid.camera_col) &&
        grid.camera_row >= 0 && grid.camera_row <= 3 && grid.camera_col >= 0 && grid.camera_col <= 3;
      row_ = grid.camera_row; col_ = grid.camera_col; pose_time_ = stamp;
      const double finished=seconds();
      if (processed_time_ > 0 && finished > processed_time_)
        vision_hz_=vision_hz_ <= 0 ? 1.0/(finished-processed_time_) :
          0.9*vision_hz_+0.1/(finished-processed_time_);
      processed_time_ = finished; ++processed_frames_;
      latency_ms_[latency_index_++ % latency_ms_.size()] = (finished-stamp)*1000.0;
      latency_count_=std::min(latency_count_+1,latency_ms_.size());
      } catch (const std::exception& e) {
        fault(std::string("vision: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }
  void send_frame(std::uint8_t type, const std::uint8_t* payload, std::size_t len) {
    std::array<std::uint8_t,AUV_PROTOCOL_MAX_FRAME_SIZE> bytes{};
    auto size = auv_protocol_encode_frame(type,payload,len,bytes.data(),bytes.size());
    if (!size || serial_.write(bytes.data(),size) != size) throw std::runtime_error("serial frame write failed");
  }
  void send_arm(bool arm) {
    std::array<std::uint8_t,5> p{};
    auv_protocol_write_u32_le(p.data(),++sequence_); p[4] = arm ? 1 : 0;
    send_frame(AUV_PROTOCOL_MSG_SET_ARMED,p.data(),p.size());
  }
  void serial_loop() {
    auv_stm32_bridge::StreamParser parser;
    auto next = Clock::now();
    while (running) {
      next += std::chrono::milliseconds(50);
      try {
        if (!serial_.is_open()) {
          if (cfg_.serial.empty()) { std::this_thread::sleep_until(next); continue; }
          serial_.open(cfg_.serial,cfg_.baud); parser.reset();
          std::lock_guard<std::mutex> lock(state_mutex_);
          serial_connected_ = false; armed_requested_ = false; disarm_pending_ = true;
          event("SERIAL_OPEN", cfg_.serial);
        }
        std::array<std::uint8_t,256> b{};
        auto n = serial_.read(b.data(),b.size());
        for (const auto& frame : parser.consume(b.data(),n)) {
          if (frame.message_type == AUV_PROTOCOL_MSG_STATUS) {
            auv_core::Stm32Status status;
            if (!auv_core::decode_status(frame.payload,status)) continue;
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_ = std::move(status); status_time_ = seconds(); serial_connected_ = true;
          } else if (frame.message_type == AUV_PROTOCOL_MSG_ACK && frame.payload.size() == 6) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const auto acknowledged_sequence=auv_protocol_read_u32_le(frame.payload.data()+2);
            if (acknowledged_sequence == arm_sequence_ && frame.payload[0] == AUV_PROTOCOL_MSG_SET_ARMED) {
              arm_ack_ = frame.payload[1] == 0;
              if (!arm_ack_) { armed_requested_ = false; disarm_pending_ = true; set_fault_locked("STM32 rejected ARM"); }
            } else if (acknowledged_sequence == gripper_command_sequence_ &&
                       frame.payload[0] == AUV_PROTOCOL_MSG_ACTUATOR_COMMAND) {
              gripper_ack_=frame.payload[1] == 0;
              if (!gripper_ack_) set_fault_locked("STM32 rejected gripper command");
            }
          } else if (frame.message_type == AUV_PROTOCOL_MSG_ACTUATOR_STATUS) {
            auv_stm32_bridge::GripperTelemetry telemetry;
            if (!auv_stm32_bridge::decode_gripper_status(frame.payload,telemetry)) continue;
            std::lock_guard<std::mutex> lock(state_mutex_);
            gripper_=telemetry; gripper_time_=seconds();
          }
        }
        const auto now = seconds();
        auv_stm32_bridge::MotionTarget motion, neutral;
        bool send_motion = false, send_disarm = false, send_neutral = false,
          send_arm_request = false, reconnect = false;
        bool control_healthy = false;
        std::optional<auv_stm32_bridge::GripperAction> gripper_command;
        std::uint32_t gripper_sequence=0;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          control_healthy = now-last_control_time_.load() <= cfg_.control_watchdog_timeout;
          if (!control_healthy) {
            set_fault_locked("control loop watchdog timeout");
            armed_requested_=false; disarm_pending_=true; motion_={};
          }
          if (serial_connected_ && !status_fresh(now)) {
            set_fault_locked("STM32 STATUS timeout"); armed_requested_ = false; disarm_pending_ = true; motion_ = {};
            if (now-status_time_ > 2*cfg_.status_timeout) { serial_connected_=false; reconnect=true; }
          }
          if (armed_requested_ && arm_time_ > 0 && now-arm_time_ > 0.5 && (!arm_ack_ || !status_.armed)) {
            set_fault_locked("ARM acknowledgement or status timeout"); armed_requested_=false; disarm_pending_=true;
          }
          if (disarm_pending_ || (serial_connected_ && status_.armed && !armed_requested_)) {
            send_disarm = true; disarm_pending_ = false;
          }
          send_neutral = send_disarm && serial_connected_ && status_.armed;
          if (status_fresh(now) && std::isfinite(status_.depth) && std::isfinite(status_.yaw)) {
            neutral.depth=status_.depth; neutral.yaw=status_.yaw;
          }
          if (arm_pending_) { send_arm_request = true; arm_pending_ = false; }
          if (gripper_pending_) {
            gripper_command=gripper_pending_;
            gripper_pending_.reset();
            gripper_sequence=++sequence_;
            gripper_command_sequence_=gripper_sequence;
            gripper_command_time_=now;
            gripper_ack_=false;
          }
          if (gripper_command_time_ > 0 && !gripper_ack_ && now-gripper_command_time_ > 0.5)
            set_fault_locked("gripper command acknowledgement timeout");
          send_motion = cfg_.motion_enabled && armed_requested_ && arm_ack_ && status_.armed &&
            safe_status(now) && propulsion_phase(mission_.snapshot().phase) && fault_.empty();
          motion = send_motion ? motion_ : auv_stm32_bridge::MotionTarget{};
        }
        if (control_healthy) {
          std::array<std::uint8_t,8> heartbeat{};
          auv_protocol_write_u32_le(heartbeat.data(),++sequence_);
          auv_protocol_write_u32_le(heartbeat.data()+4,static_cast<std::uint32_t>(now*1000));
          send_frame(AUV_PROTOCOL_MSG_HEARTBEAT,heartbeat.data(),heartbeat.size());
          ++heartbeat_count_;
        }
        if (send_disarm) {
          if (send_neutral) {
            const auto p=auv_stm32_bridge::encode_motion_target_payload(++sequence_,neutral);
            send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,p.data(),p.size());
          }
          send_arm(false);
        }
        if (send_arm_request) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (armed_requested_ && fault_.empty()) { arm_sequence_ = sequence_ + 1; arm_time_=seconds(); send_arm(true); }
        }
        if (gripper_command) {
          const auto payload=auv_stm32_bridge::encode_gripper_command(gripper_sequence,*gripper_command);
          send_frame(AUV_PROTOCOL_MSG_ACTUATOR_COMMAND,payload.data(),payload.size());
        }
        if (send_motion) {
          const auto p = auv_stm32_bridge::encode_motion_target_payload(++sequence_,motion);
          send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,p.data(),p.size());
        }
        if (reconnect) { serial_.close(); parser.reset(); }
      } catch (const std::exception& e) {
        serial_.close();
        fault(std::string("serial: ")+e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      std::this_thread::sleep_until(next);
    }
    if (serial_.is_open()) {
      try {
        auv_stm32_bridge::MotionTarget neutral;
        { std::lock_guard<std::mutex> lock(state_mutex_);
          if (status_fresh(seconds())) { neutral.depth=status_.depth; neutral.yaw=status_.yaw; } }
        if (std::isfinite(neutral.depth) && std::isfinite(neutral.yaw)) {
          const auto payload=auv_stm32_bridge::encode_motion_target_payload(++sequence_,neutral);
          send_frame(AUV_PROTOCOL_MSG_MOTION_TARGET,payload.data(),payload.size());
        }
        const auto stop_payload=auv_stm32_bridge::encode_gripper_command(
          ++sequence_,auv_stm32_bridge::GripperAction::kStop);
        send_frame(AUV_PROTOCOL_MSG_ACTUATOR_COMMAND,stop_payload.data(),stop_payload.size());
        send_arm(false);
      }
      catch (...) {}
    }
  }
  void control_loop() {
    auto next = Clock::now(); std::uint64_t old_revision = 0;
    int missed_deadlines=0;
    while (running) {
      next += std::chrono::milliseconds(50);
      const auto now = seconds();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (Clock::now() > next + std::chrono::milliseconds(100)) {
          if (++missed_deadlines >= 3) {
            set_fault_locked("control loop deadline repeatedly missed");
            armed_requested_=false; disarm_pending_=true;
          }
          next=Clock::now();
        } else missed_deadlines=0;
        auto phase = mission_.snapshot().phase;
        if (cfg_.auto_start && !autonomous_start_attempted_ && phase == auv_mission::MissionPhase::kInit) {
          const bool gripper_ready = !cfg_.mission.full_mission ||
            (gripper_status_fresh(now) && gripper_.calibrated && gripper_.error_flags == 0);
          const bool startup_ready = safe_status(now) && gripper_ready && frame_time_ > 0 &&
            now-frame_time_ <= cfg_.frame_timeout && processed_time_ > 0 &&
            now-processed_time_ <= cfg_.frame_timeout;
          if (startup_ready) {
            if (startup_ready_since_ <= 0) startup_ready_since_=now;
          } else startup_ready_since_=0;
          if (now-boot_time_ >= cfg_.startup_delay && startup_ready_since_ > 0 &&
              now-startup_ready_since_ >= cfg_.startup_stable) {
            autonomous_start_attempted_ = true;
            if (claim_autonomous_run_locked()) {
              const auto result = mission_.command(auv_mission::MissionCommand::kStart,now);
              event("AUTO_START",result.message);
              phase = mission_.snapshot().phase;
            }
          } else if (now-boot_time_ > cfg_.startup_timeout) {
            autonomous_start_attempted_ = true;
            set_fault_locked("autonomous startup readiness timeout");
            disarm_pending_ = true;
          }
        }
        if (phase != auv_mission::MissionPhase::kInit && phase != auv_mission::MissionPhase::kComplete &&
            phase != auv_mission::MissionPhase::kFault && phase != auv_mission::MissionPhase::kAborted) {
          if (!safe_status(now)) {
            set_fault_locked("STM32 unsafe or STATUS timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if (!fault_.empty()) { armed_requested_ = false; disarm_pending_ = true; }
          if (frame_time_ <= 0 || now - frame_time_ > cfg_.frame_timeout) {
            set_fault_locked("camera frame timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
          if (phase == auv_mission::MissionPhase::kVisitCones &&
              !(surface_enabled_ && armed_requested_ &&
                traverse_stage_ == TraverseStage::kSurfacing) &&
              !depth_paused_ &&
              !(traverse_stage_ == TraverseStage::kDeparting && armed_requested_) &&
              (!pose_valid_ || now - pose_time_ > cfg_.pose_timeout)) {
            set_fault_locked("grid pose timeout"); armed_requested_ = false; disarm_pending_ = true;
          }
        }
        if (status_fresh(now)) mission_.update_status(serial_connected_,status_.armed,status_.error_flags,now);
        mission_.update_apriltag(tag_found_,now);
        if (tag_found_ && !cfg_.mission.full_mission) surface_enabled_ = true;
        mission_.update_map(map_.complete, all_visited_ && departure_done_,now);
        if (phase == auv_mission::MissionPhase::kPlanCones && !route_ready_ && map_.complete &&
            (!pose_valid_ || now-pose_time_ > cfg_.pose_timeout)) {
          set_fault_locked("grid pose unavailable for planning");
          armed_requested_=false; disarm_pending_=true;
        }
        if (phase == auv_mission::MissionPhase::kPlanCones && !route_ready_ && map_.complete && fault_.empty()) {
          auv_planning::GridCell start{static_cast<std::int8_t>(std::clamp(static_cast<int>(row_),0,2)),
            static_cast<std::int8_t>(std::clamp(static_cast<int>(col_),0,2)),"unknown"};
          plan_ = planner_.plan(map_.grid,start);
          route_ready_ = plan_.valid && !plan_.targets.empty();
          if (route_ready_) { route_.set_route(plan_,++map_revision_); compute_departure(); }
          else { set_fault_locked("empty or invalid cone route: " + plan_.reason); disarm_pending_ = true; }
          std::ostringstream route_detail;
          route_detail << plan_.reason << " path=";
          for (const auto& cell : plan_.path)
            route_detail << '(' << static_cast<int>(cell.row) << ',' << static_cast<int>(cell.col) << ')';
          event("PLAN",route_detail.str());
        }
        mission_.update_route(route_ready_,route_ready_,now);
        if (gripper_status_fresh(now)) {
          mission_.update_gripper(gripper_.state == 3U,gripper_.state == 5U,now);
        }
        if (fault_.empty()) mission_.tick(now);
        else mission_.force_fault(fault_,now);
        phase = mission_.snapshot().phase;
        if (phase == auv_mission::MissionPhase::kGrab && !gripper_close_requested_) {
          if (!armed_requested_ || !status_.armed || !gripper_status_fresh(now) ||
              !gripper_.calibrated || gripper_.error_flags != 0) {
            set_fault_locked("gripper unavailable, uncalibrated, or faulted");
            mission_.force_fault(fault_,now); phase=mission_.snapshot().phase;
          } else {
            gripper_pending_=auv_stm32_bridge::GripperAction::kClose;
            gripper_close_requested_=true; event("GRIPPER_COMMAND","close");
          }
        }
        if (phase == auv_mission::MissionPhase::kRelease && !gripper_open_requested_) {
          if (!armed_requested_ || !status_.armed || !gripper_status_fresh(now) ||
              !gripper_.calibrated || gripper_.error_flags != 0) {
            set_fault_locked("gripper unavailable, uncalibrated, or faulted");
            mission_.force_fault(fault_,now); phase=mission_.snapshot().phase;
          } else {
            gripper_pending_=auv_stm32_bridge::GripperAction::kOpen;
            gripper_open_requested_=true; event("GRIPPER_COMMAND","open");
          }
        }
        if (cfg_.auto_arm && !autonomous_arm_attempted_ &&
            phase == auv_mission::MissionPhase::kVisitCones) {
          autonomous_arm_attempted_ = true;
          if (arm_gate_ready(now)) request_arm_locked("autonomous");
          else {
            set_fault_locked("autonomous ARM safety gate rejected");
            armed_requested_ = false; disarm_pending_ = true;
            mission_.force_fault(fault_,now);
            phase = mission_.snapshot().phase;
          }
        }
        if (!fault_.empty() || phase == auv_mission::MissionPhase::kFault) {
          armed_requested_ = false; disarm_pending_ = true; motion_ = {};
          if (!gripper_stop_requested_) {
            gripper_pending_=auv_stm32_bridge::GripperAction::kStop;
            gripper_stop_requested_=true;
          }
        } else {
          const bool visiting = phase == auv_mission::MissionPhase::kVisitCones;
          const bool surfacing = surface_enabled_ && visiting &&
            traverse_stage_ == TraverseStage::kSurfacing;
          const bool departing = visiting && traverse_stage_ == TraverseStage::kDeparting;
          const bool armed_moving = cfg_.motion_enabled && armed_requested_ && arm_ack_ &&
            status_.armed && safe_status(now);
          const double target_depth = surface_enabled_ ? cfg_.surface_depth_m : hold_depth_;

          // Depth hold watchdog (traversal + departure): pause horizontal motion
          // if the depth leaves the dead zone around target_depth so the STM32
          // depth PID can recover it; resume once it stays in band for the
          // configured stable time. Escalates to FAULT after depth_pause_timeout_sec.
          if (armed_moving &&
              (traverse_stage_ == TraverseStage::kTraversing ||
               traverse_stage_ == TraverseStage::kDeparting) &&
              status_fresh(now) && std::isfinite(status_.depth)) {
            const double err = std::fabs(status_.depth - target_depth);
            if (!depth_paused_) {
              if (err > cfg_.depth_deadzone_m) {
                depth_paused_ = true; depth_paused_since_ = now; depth_in_band_since_ = 0;
              }
            } else {
              if (err <= cfg_.depth_deadzone_m) {
                if (depth_in_band_since_ <= 0) depth_in_band_since_ = now;
                else if (now - depth_in_band_since_ >= cfg_.depth_recover_stable_sec)
                  depth_paused_ = false;
              } else {
                depth_in_band_since_ = 0;
              }
              if (now - depth_paused_since_ > cfg_.depth_pause_timeout_sec) {
                set_fault_locked("depth hold timeout (cone collision risk)");
                armed_requested_ = false; disarm_pending_ = true; motion_ = {};
              }
            }
          } else if (depth_paused_) {
            depth_paused_ = false; depth_in_band_since_ = 0; depth_paused_since_ = 0;
          }

          const bool hold_route = surfacing || departing || depth_paused_;
          route_.set_mission_active(visiting && !hold_route);
          route_.set_vehicle_ready(armed_moving);
          route_.set_pose(pose_valid_ && now-pose_time_ <= cfg_.pose_timeout,row_,col_);
          const auto step = route_.step();
          waypoint_index_ = step.waypoint_index;
          if (visiting && step.state == auv_control::RouteStep::State::kFault)
            { set_fault_locked(step.detail); armed_requested_ = false; disarm_pending_ = true; }
          if (step.visited_cell) {
            auto i = static_cast<std::size_t>(step.visited_cell->row*3+step.visited_cell->col);
            visited_[i] = true; event("CONE_VISITED",std::to_string(i));
          }
          all_visited_ = !plan_.targets.empty();
          for (const auto& t : plan_.targets) all_visited_ &= visited_[static_cast<std::size_t>(t.row*3+t.col)];

          // After every cone is visited, leave the 3x3 grid along the precomputed
          // exit direction (blind, fixed duration), then report completion.
          if (visiting && traverse_stage_ == TraverseStage::kTraversing &&
              all_visited_ && armed_requested_ && fault_.empty()) {
            traverse_stage_ = TraverseStage::kDeparting;
            departing_started_ = 0;
            std::ostringstream detail;
            detail << "exit=(" << exit_dr_ << ',' << exit_dc_ << ')';
            event("DEPARTING", detail.str());
          }

          motion_ = {};
          if (armed_requested_) {
            motion_.yaw = hold_yaw_;
            motion_.depth = target_depth;
          }
          if (surfacing && armed_moving) {
            if (surfacing_started_ <= 0) surfacing_started_ = now;
            if (status_fresh(now) && std::isfinite(status_.depth) &&
                std::fabs(status_.depth - cfg_.surface_depth_m) <= cfg_.surface_tolerance_m) {
              if (surfacing_since_ <= 0) surfacing_since_ = now;
              else if (now - surfacing_since_ >= cfg_.surface_stable_sec) {
                traverse_stage_ = TraverseStage::kTraversing;
                std::ostringstream detail;
                detail << "depth=" << status_.depth << " m";
                event("SURFACED", detail.str());
              }
            } else {
              surfacing_since_ = 0;
            }
            if (surfacing_started_ > 0 && now - surfacing_started_ > cfg_.surface_timeout_sec) {
              set_fault_locked("surface timeout");
              armed_requested_ = false; disarm_pending_ = true; motion_ = {};
            }
          } else if (departing && armed_moving) {
            if (departing_started_ <= 0) departing_started_ = now;
            if (!depth_paused_) {
              motion_.vx = depart_surge_; motion_.vy = depart_sway_;
            }
            if (now - departing_started_ >= cfg_.depart_duration_sec)
              departure_done_ = true;
          } else if (step.state == auv_control::RouteStep::State::kRunning && armed_requested_ && fault_.empty() && !depth_paused_) {
            motion_.vx = static_cast<float>(step.surge); motion_.vy = static_cast<float>(step.sway);
            motion_.depth = target_depth;
            motion_.yaw = hold_yaw_;
          }
        }
        const auto snapshot = mission_.snapshot();
        ++control_ticks_;
        if (snapshot.revision != old_revision) { old_revision = snapshot.revision; event("MISSION",auv_mission::mission_phase_name(snapshot.phase)); }
        if (now-last_state_log_sec_ >= 1.0) {
          last_state_log_sec_=now;
          std::ostringstream detail;
          detail << auv_mission::mission_phase_name(snapshot.phase)
                 << " serial=" << serial_connected_ << " armed=" << status_.armed
                 << " grid=" << map_.complete << " pose=" << pose_valid_
                 << " waypoint=" << waypoint_index_;
          event("STATE",detail.str());
        }
        if (snapshot.phase == auv_mission::MissionPhase::kComplete) { armed_requested_ = false; disarm_pending_ = true; }
        last_control_time_=seconds();
      }
      std::this_thread::sleep_until(next);
    }
  }
  std::string command(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = seconds();
    if (cmd == "status") {
      std::ostringstream s;
      auto latency=std::vector<double>(latency_ms_.begin(),latency_ms_.begin()+latency_count_);
      std::sort(latency.begin(),latency.end());
      double p99=latency.empty() ? -1.0 : latency[static_cast<std::size_t>(0.99*(latency.size()-1))];
      s << "{\"phase\":\"" << auv_mission::mission_phase_name(mission_.snapshot().phase)
        << "\",\"operation_mode\":\"" << cfg_.operation_mode
        << "\",\"mission_profile\":\"" << cfg_.mission_profile
        << "\",\"auto_start\":" << (cfg_.auto_start ? "true":"false")
        << ",\"auto_arm\":" << (cfg_.auto_arm ? "true":"false")
        << ",\"serial\":" << (serial_connected_ ? "true":"false")
        << ",\"armed\":" << (status_.armed ? "true":"false")
        << ",\"motion_enabled\":" << (cfg_.motion_enabled ? "true":"false")
        << ",\"telemetry_valid\":" << (status_.telemetry_valid ? "true":"false")
        << ",\"voltage_valid\":" << (status_.voltage_valid ? "true":"false")
        << ",\"camera_age_sec\":" << (frame_time_ ? now-frame_time_ : -1)
        << ",\"down_source\":\"" << json_escape(cfg_.camera) << "\""
        << ",\"front_source\":\"" << json_escape(cfg_.front_source) << "\""
        << ",\"down_capture_frames\":" << down_capture_frames_.load()
        << ",\"down_hz\":" << down_hz_.load()
        << ",\"video_enabled\":" << (cfg_.video_enabled ? "true":"false")
        << ",\"front_enabled\":" << (cfg_.front_enabled ? "true":"false")
        << ",\"front_camera_age_sec\":" << (front_time_ ? now-front_time_ : -1)
        << ",\"front_frames\":" << front_frames_
        << ",\"front_hz\":" << front_hz_
        << ",\"front_degraded\":" << (cfg_.front_enabled &&
            (!front_time_ || now-front_time_>cfg_.frame_timeout || !front_detail_.empty()) ? "true":"false")
        << ",\"front_detail\":\"" << json_escape(front_detail_) << "\""
        << ",\"vision_frames\":" << processed_frames_
        << ",\"vision_hz\":" << vision_hz_
        << ",\"vision_latency_p99_ms\":" << p99
        << ",\"control_ticks\":" << control_ticks_
        << ",\"control_age_sec\":" << now-last_control_time_.load()
        << ",\"heartbeats\":" << heartbeat_count_.load()
        << ",\"vision_age_sec\":" << (processed_time_ ? now-processed_time_ : -1)
        << ",\"grid_complete\":" << (map_.complete ? "true":"false")
        << ",\"apriltag_found\":" << (tag_found_ ? "true":"false")
        << ",\"cone_count\":" << map_.cone_count << ",\"route_waypoints\":" << plan_.path.size()
        << ",\"pose_valid\":" << (pose_valid_ ? "true":"false")
        << ",\"surfacing\":" << (surface_enabled_ && traverse_stage_ == TraverseStage::kSurfacing ? "true":"false")
        << ",\"departing\":" << (traverse_stage_ == TraverseStage::kDeparting ? "true":"false")
        << ",\"depth_paused\":" << (depth_paused_ ? "true":"false")
        << ",\"surface_depth_m\":" << cfg_.surface_depth_m
        << ",\"row\":" << (std::isfinite(row_) ? row_ : -1.0F)
        << ",\"col\":" << (std::isfinite(col_) ? col_ : -1.0F)
        << ",\"next_waypoint_index\":" << waypoint_index_
        << ",\"next_row\":" << (waypoint_index_ < plan_.path.size() ? static_cast<int>(plan_.path[waypoint_index_].row) : -1)
        << ",\"next_col\":" << (waypoint_index_ < plan_.path.size() ? static_cast<int>(plan_.path[waypoint_index_].col) : -1)
        << ",\"error_flags\":" << status_.error_flags
        << ",\"gripper_fresh\":" << (gripper_status_fresh(now) ? "true":"false")
        << ",\"gripper_calibrated\":" << (gripper_.calibrated ? "true":"false")
        << ",\"gripper_state\":" << static_cast<unsigned>(gripper_.state)
        << ",\"gripper_error_flags\":" << static_cast<unsigned>(gripper_.error_flags)
        << ",\"cells\":[";
      for (std::size_t i=0;i<map_.grid.cells.size();++i) {
        if (i) s << ',';
        const auto& cell=map_.grid.cells[i];
        s << "{\"row\":" << static_cast<int>(cell.cell.row)
          << ",\"col\":" << static_cast<int>(cell.cell.col)
          << ",\"object\":\"" << json_escape(cell.cell.object_type)
          << "\",\"visited\":" << (visited_[i] ? "true":"false") << '}';
      }
      s << "]"
        << ",\"video_degraded\":" << (video_detail_.empty() ? "false":"true")
        << ",\"log_degraded\":" << (log_degraded_ ? "true":"false")
        << ",\"video_detail\":\"" << json_escape(video_detail_)
        << "\",\"web_detail\":\"" << json_escape(web_detail_)
        << "\",\"fault\":\"" << json_escape(fault_) << "\"}\n";
      return s.str();
    }
    if (cfg_.operation_mode == "autonomous" && cmd != "status" && cmd != "disarm") {
      event("COMMAND_REJECTED",cmd+": autonomous mode");
      return "ERR operator command disabled in autonomous mode\n";
    }
    if (cmd == "disarm" || cmd == "abort" || cmd == "pause" || cmd == "reset") {
      armed_requested_ = false; arm_ack_ = false; arm_pending_ = false;
      arm_time_=0;
      disarm_pending_ = true; motion_ = {};
      gripper_pending_=auv_stm32_bridge::GripperAction::kStop;
      gripper_stop_requested_=true;
    }
    if (cmd == "arm SAFE_TO_ARM") {
      if (!cfg_.motion_enabled || !cfg_.directions_calibrated || !cfg_.limits_calibrated || cfg_.serial.empty()) return "ERR motion configuration disabled or uncalibrated\n";
      if (!arm_gate_ready(now))
        return "ERR ARM safety gate rejected\n";
      request_arm_locked("operator"); return "OK ARM requested\n";
    }
    if (cmd == "arm") return "ERR use arm --confirm SAFE_TO_ARM\n";
    if (cmd == "disarm") {
      event("DISARM", "operator");
      if (cfg_.operation_mode == "autonomous") {
        set_fault_locked("emergency DISARM requested");
        mission_.force_fault(fault_,now);
      }
      return "OK DISARM requested\n";
    }
    std::optional<auv_mission::MissionCommand> c;
    if (cmd == "start") c = auv_mission::MissionCommand::kStart;
    if (cmd == "pause") c = auv_mission::MissionCommand::kPause;
    if (cmd == "resume") c = auv_mission::MissionCommand::kResume;
    if (cmd == "abort") c = auv_mission::MissionCommand::kAbort;
    if (cmd == "reset") c = auv_mission::MissionCommand::kReset;
    if (!c) return "ERR unknown command\n";
    auto r = mission_.command(*c,now);
    if (cmd == "reset" && r.accepted) {
      fault_.clear(); route_.reset(); route_ready_=false; plan_={}; map_={};
      visited_.fill(false); all_visited_=false; tag_found_=false;
      pose_valid_=false; pose_time_=0; waypoint_index_=0;
      traverse_stage_ = TraverseStage::kSurfacing;
      surfacing_started_ = 0; surfacing_since_ = 0;
      surface_enabled_ = cfg_.surface_before_traversal;
      departing_started_ = 0; departure_done_ = false;
      depth_paused_ = false; depth_paused_since_ = 0; depth_in_band_since_ = 0;
      exit_dr_ = 0; exit_dc_ = 0; depart_surge_ = 0.0F; depart_sway_ = 0.0F;
      gripper_close_requested_=false; gripper_open_requested_=false;
      gripper_stop_requested_=false;
      gripper_pending_.reset(); gripper_ack_=false; gripper_command_time_=0;
      map_image_saved_=false;
    }
    event("COMMAND",cmd+": "+r.message);
    return std::string(r.accepted ? "OK ":"ERR ")+r.message+"\n";
  }
  void video_loop() {
    if (!cfg_.video_enabled) return;
    std::filesystem::create_directories(cfg_.video_dir);
    std::uint64_t seen = 0;
    int pipefd[2];
    if (::pipe2(pipefd,O_CLOEXEC) != 0) { std::lock_guard<std::mutex> lock(state_mutex_); video_detail_="pipe failed"; return; }
    const std::string output=cfg_.video_dir+"/index.m3u8";
    const std::string size=std::to_string(cfg_.video_width)+"x"+std::to_string(cfg_.video_height);
    const std::string fps=std::to_string(cfg_.video_fps);
    const std::string bitrate=std::to_string(cfg_.video_bitrate_kbps)+"k";
    const std::string gop=std::to_string(std::max(1,static_cast<int>(cfg_.video_fps*cfg_.segment_time)));
    const std::string segment=std::to_string(cfg_.segment_time);
    std::vector<std::string> args={"ffmpeg","-hide_banner","-loglevel","error","-nostdin","-y",
      "-filter_threads","1","-f","rawvideo","-pixel_format","bgr24","-video_size",size,"-framerate",fps,
      "-i","pipe:0","-an","-c:v",cfg_.video_encoder,"-b:v",bitrate,"-g",gop,"-keyint_min",gop,"-sc_threshold","0"};
    if (cfg_.video_encoder=="libx264")
      args.insert(args.end(),{"-preset","ultrafast","-tune","zerolatency","-threads","1"});
    args.insert(args.end(),{"-bsf:v","extract_extradata,dump_extra=freq=keyframe","-f","hls",
      "-hls_time",segment,"-hls_list_size","6","-hls_flags","delete_segments+independent_segments",output});
    std::vector<char*> argv;
    for (auto& arg:args) argv.push_back(arg.data());
    argv.push_back(nullptr);
    pid_t child = ::fork();
    if (child == 0) {
      ::dup2(pipefd[0],STDIN_FILENO); ::close(pipefd[0]); ::close(pipefd[1]);
      ::execvp(argv[0],argv.data());
      _exit(127);
    }
    ::close(pipefd[0]);
    if (child < 0) { ::close(pipefd[1]); std::lock_guard<std::mutex> lock(state_mutex_); video_detail_="fork failed"; return; }
    auto next=Clock::now();
    while (running) {
      next+=std::chrono::milliseconds(1000/cfg_.video_fps);
      int exit_status=0;
      if (::waitpid(child,&exit_status,WNOHANG) == child) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        video_detail_="FFmpeg unavailable or hardware encoder failed";
        running_video_=false;
        break;
      }
      cv::Mat image;
      /* Preview consumes capture directly; perception rate must not cap video FPS. */
      { std::lock_guard<std::mutex> lock(frame_mutex_);
        if (cfg_.front_enabled || seen != frame_sequence_) { seen=frame_sequence_; image=frame_; } }
      if (cfg_.front_enabled) {
        cv::Mat front;
        { std::lock_guard<std::mutex> lock(front_mutex_); front=front_frame_; }
        cv::Mat composite(cfg_.video_height,cfg_.video_width,CV_8UC3,cv::Scalar(0,0,0));
        const int half=cfg_.video_width/2;
        if (!image.empty() && frame_time_>0 && seconds()-frame_time_<=cfg_.frame_timeout)
          cv::resize(image,composite(cv::Rect(0,0,half,cfg_.video_height)),{half,cfg_.video_height});
        if (!front.empty() && front_time_>0 && seconds()-front_time_<=cfg_.frame_timeout)
          cv::resize(front,composite(cv::Rect(half,0,cfg_.video_width-half,cfg_.video_height)),
            {cfg_.video_width-half,cfg_.video_height});
        cv::putText(composite,"DOWN / CSI",{8,20},cv::FONT_HERSHEY_SIMPLEX,.45,{0,255,255},1);
        cv::putText(composite,"FRONT / USB",{half+8,20},cv::FONT_HERSHEY_SIMPLEX,.45,{0,255,255},1);
        image=std::move(composite);
      }
      if (image.empty() && (frame_time_ <= 0 || seconds()-frame_time_ > cfg_.frame_timeout)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        video_detail_="waiting for down camera frames";
      }
      if (!image.empty()) {
        cv::Mat resized;
        if (image.cols != cfg_.video_width || image.rows != cfg_.video_height)
          cv::resize(image,resized,{cfg_.video_width,cfg_.video_height});
        else resized=image;
        if (!resized.isContinuous()) resized=resized.clone();
        const auto* bytes=resized.ptr<std::uint8_t>();
        std::size_t size=static_cast<std::size_t>(cfg_.video_width)*cfg_.video_height*3U,offset=0;
        while (offset<size && running) {
          auto n=::write(pipefd[1],bytes+offset,size-offset);
          if (n <= 0) { std::lock_guard<std::mutex> lock(state_mutex_); video_detail_="FFmpeg encoder failed"; running_video_=false; break; }
          offset+=static_cast<std::size_t>(n);
        }
        if (!running_video_) break;
        { std::lock_guard<std::mutex> lock(state_mutex_);
          if (video_detail_ == "waiting for down camera frames") video_detail_.clear(); }
      }
      if (next < Clock::now()) next=Clock::now(); /* No catch-up frame bursts. */
      std::this_thread::sleep_until(next);
    }
    ::close(pipefd[1]);
    int status=0; ::waitpid(child,&status,0);
    if (running && cfg_.software_fallback && cfg_.video_encoder != "libx264") {
      std::lock_guard<std::mutex> lock(state_mutex_);
      video_detail_="hardware encoder failed; switching to explicitly enabled libx264";
      cfg_.video_encoder="libx264";
      running_video_=true;
    }
    if (running && cfg_.software_fallback && cfg_.video_encoder == "libx264" && running_video_)
      video_loop();
  }
#ifdef AUV_HAVE_HTTPLIB
  void web_loop() {
    if (!running) return;
    httplib::Server server;
    web_server_=&server;
    server.Get("/api/localization",[this](const httplib::Request&,httplib::Response& res){
      res.set_header("Cache-Control","no-store");res.set_content(localization_->json(seconds()),"application/json");
    });
    server.Post("/api/localization/reset",[this](const httplib::Request&,httplib::Response& res){
      if(!localization_->reset(seconds())){res.status=409;res.set_content("{\"error\":\"Require calibrated fresh input, DISARM and 2 seconds stable attitude/depth\"}","application/json");return;}
      event("LOCALIZATION_RESET",localization_->json(seconds()));
      res.set_content(localization_->json(seconds()),"application/json");
    });
    server.Get("/api/status",[this](const httplib::Request&,httplib::Response& res){
      res.set_content(command("status"),"application/json");
      res.set_header("Cache-Control","no-store");
    });
    auto cached=[this](bool front,std::vector<std::uint8_t>& jpeg,double& stamp) {
      if(front) { std::lock_guard<std::mutex> lock(front_mutex_); jpeg=front_jpeg_; stamp=front_time_.load(); }
      else { std::lock_guard<std::mutex> lock(frame_mutex_); jpeg=down_jpeg_; stamp=frame_time_.load(); }
      return !jpeg.empty() && stamp>0 && seconds()-stamp<=cfg_.frame_timeout;
    };
    auto snapshot=[this,cached](bool front,httplib::Response& res) {
      std::vector<std::uint8_t> jpeg;
      double stamp;
      if(!cached(front,jpeg,stamp)) { res.status=503; return; }
      res.set_content(reinterpret_cast<const char*>(jpeg.data()),jpeg.size(),"image/jpeg");
      res.set_header("Cache-Control","no-store");
      res.set_header("X-Frame-Time-Monotonic",std::to_string(stamp));
      res.set_header("X-Camera-Source",front ? cfg_.front_source : cfg_.camera);
    };
    server.Get("/api/camera/down.jpg",[snapshot](const httplib::Request&,httplib::Response& res){snapshot(false,res);});
    server.Get("/api/camera/front.jpg",[snapshot](const httplib::Request&,httplib::Response& res){snapshot(true,res);});
    server.new_task_queue=[] { return new httplib::ThreadPool(8); };
    server.set_write_timeout(2,0);
    server.set_tcp_nodelay(true);
    auto stream=[this,cached](bool front,httplib::Response& res) {
      /* Reserve worker capacity for status and control requests. */
      if (mjpeg_clients_.fetch_add(1)>=4) {
        --mjpeg_clients_; res.status=429; return;
      }
      res.set_header("Cache-Control","no-store");
      res.set_chunked_content_provider("multipart/x-mixed-replace; boundary=auvframe",
        [this,cached,front,last=-1.0,delivered=seconds()](size_t,httplib::DataSink& sink) mutable {
          while (running && sink.is_writable()) {
            std::vector<std::uint8_t> jpeg;
            double stamp;
            if (cached(front,jpeg,stamp) && stamp!=last) {
              last=stamp; delivered=seconds();
              const auto header=std::string("--auvframe\r\nContent-Type: image/jpeg\r\nContent-Length: ")+
                std::to_string(jpeg.size())+"\r\nX-Frame-Time-Monotonic: "+std::to_string(stamp)+
                "\r\nX-Camera-Source: "+(front ? cfg_.front_source : cfg_.camera)+"\r\n\r\n";
              std::string part;
              part.reserve(header.size()+jpeg.size()+2);
              part.append(header);
              part.append(reinterpret_cast<const char*>(jpeg.data()),jpeg.size());
              part.append("\r\n");
              return sink.write(part.data(),part.size());
            }
            if (seconds()-delivered>2.0) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
          return false;
        },[this](bool) { --mjpeg_clients_; });
    };
    server.Get("/api/camera/down.mjpeg",[stream](const httplib::Request&,httplib::Response& res){stream(false,res);});
    server.Get("/api/camera/front.mjpeg",[stream](const httplib::Request&,httplib::Response& res){stream(true,res);});
    server.Get("/",[this](const httplib::Request&,httplib::Response& res){
      std::ifstream in(cfg_.web_assets+"/index.html");
      if (!in) { res.status=503; return; }
      res.set_content(std::string(std::istreambuf_iterator<char>(in),{}),"text/html; charset=utf-8");
    });
    server.Get("/hls.min.js",[this](const httplib::Request&,httplib::Response& res){
      std::ifstream in(cfg_.web_assets+"/hls.min.js",std::ios::binary);
      if (!in) { res.status=503; return; }
      res.set_content(std::string(std::istreambuf_iterator<char>(in),{}),"application/javascript");
    });
    server.set_mount_point("/hls",cfg_.video_dir);
    if (!server.listen(cfg_.web_bind, cfg_.web_port) && running) {
      std::lock_guard<std::mutex> lock(state_mutex_); web_detail_="HTTP bind failed";
    }
    web_server_=nullptr;
  }
#endif
  void socket_loop() {
    std::filesystem::create_directories(std::filesystem::path(cfg_.socket).parent_path());
    int fd = ::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);
    if (fd < 0) throw std::runtime_error("control socket failed");
    sockaddr_un addr{}; addr.sun_family = AF_UNIX;
    if (cfg_.socket.size() >= sizeof(addr.sun_path)) throw std::runtime_error("control socket path too long");
    std::strncpy(addr.sun_path,cfg_.socket.c_str(),sizeof(addr.sun_path)-1);
    ::unlink(cfg_.socket.c_str());
    if (::bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr)) || ::listen(fd,8)) throw std::runtime_error("control socket bind failed");
    ::chmod(cfg_.socket.c_str(),0660);
    timeval timeout{0,200000}; ::setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    while (running) {
      int peer = ::accept4(fd,nullptr,nullptr,SOCK_CLOEXEC);
      if (peer < 0) continue;
      char b[128]{}; const auto n = ::read(peer,b,sizeof(b)-1);
      if (n > 0) { std::string request(b,static_cast<std::size_t>(n));
        request.erase(request.find_last_not_of("\r\n ")+1);
        const auto answer = command(request);
        std::size_t offset=0;
        while (offset < answer.size()) {
          const auto written=::write(peer,answer.data()+offset,answer.size()-offset);
          if (written <= 0) break;
          offset+=static_cast<std::size_t>(written);
        }
      }
      ::close(peer);
    }
    ::close(fd); ::unlink(cfg_.socket.c_str());
  }
  Config cfg_;
  std::atomic<double> down_hz_{0};
  std::atomic<int> mjpeg_clients_{0};
  std::vector<std::uint8_t> down_jpeg_,front_jpeg_;
  std::mutex front_mutex_;
  cv::Mat front_frame_;
  std::atomic<double> front_time_{0};
  std::atomic<std::uint64_t> down_capture_frames_{0};
  std::uint64_t front_frames_{};
  double front_hz_{};
  std::string front_detail_;
  std::mutex frame_mutex_,video_mutex_,state_mutex_,log_mutex_;
  std::condition_variable log_cv_;
  std::deque<std::string> log_queue_;
  std::atomic<bool> log_degraded_{false};
  std::atomic<bool> log_stop_{false};
  double log_started_{seconds()};
  cv::Mat frame_; std::atomic<double> frame_time_{0}; double pose_time_{},processed_time_{},status_time_{},gripper_time_{};
  cv::Mat video_frame_;
  std::uint64_t video_sequence_{};
  double vision_hz_{};
  std::array<double,256> latency_ms_{};
  std::size_t latency_index_{},latency_count_{};
  std::uint64_t control_ticks_{};
  std::atomic<std::uint64_t> heartbeat_count_{0};
  std::atomic<bool> map_image_saved_{false};
  double last_state_log_sec_{};
  std::uint64_t frame_sequence_{},processed_frames_{};
  auv_mission::MissionFsm mission_;
  auv_control::RouteExecutor route_;
  auv_planning::GridPlanner planner_;
  auv_core::SemanticMap map_;
  auv_core::Stm32Status status_;
  auv_stm32_bridge::GripperTelemetry gripper_;
  auv_planning::PlanResult plan_;
  std::array<bool,9> visited_{};
  auv_stm32_bridge::MotionTarget motion_{};
  auv_stm32_bridge::SerialPort serial_;
  std::uint32_t sequence_{},arm_sequence_{},gripper_command_sequence_{},map_revision_{};
  double arm_time_{};
  double boot_time_{seconds()};
  double startup_ready_since_{};
  double gripper_command_time_{};
  std::atomic<double> last_control_time_{seconds()};
  float hold_depth_{},hold_yaw_{};
  TraverseStage traverse_stage_{TraverseStage::kSurfacing};
  double surfacing_started_{0},surfacing_since_{0};
  bool surface_enabled_{false};
  double departing_started_{0};
  bool departure_done_{false};
  bool depth_paused_{false};
  double depth_paused_since_{0}, depth_in_band_since_{0};
  int exit_dr_{0}, exit_dc_{0};
  float depart_surge_{0.0F}, depart_sway_{0.0F};
  std::size_t waypoint_index_{};
  std::unique_ptr<Localization> localization_;
  bool tag_found_{},pose_valid_{},route_ready_{},all_visited_{},serial_connected_{},armed_requested_{},arm_ack_{},arm_pending_{},disarm_pending_{true};
  bool autonomous_start_attempted_{},autonomous_arm_attempted_{};
  bool gripper_ack_{},gripper_close_requested_{},gripper_open_requested_{},gripper_stop_requested_{};
  std::optional<auv_stm32_bridge::GripperAction> gripper_pending_;
  float row_{},col_{};
  std::string fault_,video_detail_,web_detail_;
  bool running_video_{true};
#ifdef AUV_HAVE_HTTPLIB
  std::atomic<httplib::Server*> web_server_{nullptr};
  std::atomic<bool> web_finished_{false};
#endif
  std::ofstream log_;
};
int main(int argc,char** argv) {
  try { if (argc != 2) { std::cerr << "usage: auv_runtime CONFIG.yaml\n"; return 2; }
    // This process already has dedicated capture, vision and control threads.
    // Avoid OpenCV worker-pool oversubscription on the Raspberry Pi.
    cv::setNumThreads(1);
    Runtime(load_config(argv[1])).run(); return 0;
  } catch (const std::exception& e) { std::cerr << "auv_runtime: " << e.what() << '\n'; return 1; }
}
