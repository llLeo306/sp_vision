#include "SpVisionTracker.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <utility>

#include "logger.hpp"
#include "tools/img_tools.hpp"

namespace
{
std::string VecToYaml(const std::vector<double> & v)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << v[i];
  }
  oss << "]";
  return oss.str();
}

double ArmorReprojectionError(
  const std::vector<cv::Point2f> & measured, const std::vector<cv::Point2f> & projected)
{
  if (measured.size() != 4 || projected.size() != 4) {
    return std::numeric_limits<double>::infinity();
  }
  // Corner order from detector may rotate/flip between frames; use best cyclic/reversed assignment.
  double best = std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < 4; ++shift) {
    double err_forward = 0.0;
    double err_reverse = 0.0;
    for (int i = 0; i < 4; ++i) {
      const int j_forward = (i + shift) % 4;
      const int j_reverse = (shift - i + 8) % 4;
      err_forward +=
        cv::norm(measured[static_cast<size_t>(i)] - projected[static_cast<size_t>(j_forward)]);
      err_reverse +=
        cv::norm(measured[static_cast<size_t>(i)] - projected[static_cast<size_t>(j_reverse)]);
    }
    best = std::min(best, std::min(err_forward, err_reverse));
  }
  return best / 4.0;
}

cv::Point2f QuadCenter(const std::vector<cv::Point2f> & pts)
{
  if (pts.empty()) {
    return {};
  }
  cv::Point2f c(0.0f, 0.0f);
  for (const auto & p : pts) {
    c += p;
  }
  c *= (1.0f / static_cast<float>(pts.size()));
  return c;
}

cv::Rect2f QuadBBox(const std::vector<cv::Point2f> & pts)
{
  if (pts.size() != 4) {
    return {};
  }
  return cv::boundingRect(pts);
}

double RectIou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  if (a.area() <= 0.0f || b.area() <= 0.0f) {
    return 0.0;
  }
  const cv::Rect2f inter = a & b;
  if (inter.area() <= 0.0f) {
    return 0.0;
  }
  const float uni = a.area() + b.area() - inter.area();
  if (uni <= 1e-6f) {
    return 0.0;
  }
  return static_cast<double>(inter.area() / uni);
}

double AbsAngleDiff(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

double QuadPolyIou(const std::vector<cv::Point2f> & a, const std::vector<cv::Point2f> & b)
{
  if (a.size() != 4 || b.size() != 4) {
    return 0.0;
  }
  const double area_a = std::abs(cv::contourArea(a));
  const double area_b = std::abs(cv::contourArea(b));
  if (area_a <= 1e-6 || area_b <= 1e-6) {
    return 0.0;
  }
  std::vector<cv::Point2f> inter;
  const float inter_area = cv::intersectConvexConvex(a, b, inter, true);
  if (inter_area <= 1e-6f) {
    return 0.0;
  }
  const double uni = area_a + area_b - static_cast<double>(inter_area);
  if (uni <= 1e-6) {
    return 0.0;
  }
  return static_cast<double>(inter_area) / uni;
}

struct SetOverlapScore
{
  double sum = 0.0;
  int match_count = 0;
};

// Maximum-sum bipartite matching (small-set DP) for per-frame IoU evaluation.
SetOverlapScore MaxIoUSumDP(const std::vector<std::vector<double>> & w)
{
  const int left = static_cast<int>(w.size());
  if (left == 0) {
    return {};
  }
  const int right = static_cast<int>(w.front().size());
  if (right == 0 || right > 20) {  // Safety bound for bitmask DP.
    return {};
  }

  struct State
  {
    double sum = -1e30;
    int cnt = -1;
  };

  const int mask_size = 1 << right;
  std::vector<State> dp(mask_size), ndp(mask_size);
  dp[0] = {0.0, 0};

  auto better = [](const State & a, const State & b) {
    if (a.sum > b.sum + 1e-9) return true;
    if (b.sum > a.sum + 1e-9) return false;
    return a.cnt > b.cnt;
  };

  for (int i = 0; i < left; ++i) {
    std::fill(ndp.begin(), ndp.end(), State{});
    for (int mask = 0; mask < mask_size; ++mask) {
      if (dp[mask].cnt < 0) {
        continue;
      }
      if (better(dp[mask], ndp[mask])) {
        ndp[mask] = dp[mask];  // Skip current left node.
      }
      for (int j = 0; j < right; ++j) {
        if ((mask & (1 << j)) != 0) {
          continue;
        }
        const int nmask = mask | (1 << j);
        State cand = dp[mask];
        cand.sum += std::max(0.0, w[i][j]);
        cand.cnt += 1;
        if (better(cand, ndp[nmask])) {
          ndp[nmask] = cand;
        }
      }
    }
    dp.swap(ndp);
  }

  State best{};
  best.sum = -1e30;
  best.cnt = -1;
  for (const auto & s : dp) {
    if (better(s, best)) {
      best = s;
    }
  }
  if (best.cnt < 0) {
    return {};
  }
  return {best.sum, best.cnt};
}

struct SetOverlapMetrics
{
  double iou_norm = std::numeric_limits<double>::quiet_NaN();  // sum IoU / pred_count
  double iou_visible = std::numeric_limits<double>::quiet_NaN();  // sum IoU / obs_count
  double iou_match_mean = std::numeric_limits<double>::quiet_NaN();
  double coverage = std::numeric_limits<double>::quiet_NaN();  // matched / pred_count
  double visible_recall = std::numeric_limits<double>::quiet_NaN();  // matched / obs_count
  int pred_count = 0;
  int obs_count = 0;
  int match_count = 0;
};

SetOverlapMetrics EvaluateSetOverlap(
  const std::vector<std::vector<cv::Point2f>> & preds,
  const std::vector<std::vector<cv::Point2f>> & obs)
{
  SetOverlapMetrics out;
  out.pred_count = static_cast<int>(preds.size());
  out.obs_count = static_cast<int>(obs.size());
  if (out.pred_count == 0 || out.obs_count == 0) {
    out.iou_norm = 0.0;
    out.coverage = 0.0;
    return out;
  }

  std::vector<std::vector<double>> w(out.pred_count, std::vector<double>(out.obs_count, 0.0));
  for (int i = 0; i < out.pred_count; ++i) {
    for (int j = 0; j < out.obs_count; ++j) {
      w[i][j] = QuadPolyIou(preds[i], obs[j]);
    }
  }

  const SetOverlapScore best = MaxIoUSumDP(w);
  out.match_count = best.match_count;
  out.iou_norm = best.sum / static_cast<double>(std::max(1, out.pred_count));
  out.iou_visible = best.sum / static_cast<double>(std::max(1, out.obs_count));
  out.coverage = static_cast<double>(out.match_count) / static_cast<double>(std::max(1, out.pred_count));
  out.visible_recall =
    static_cast<double>(out.match_count) / static_cast<double>(std::max(1, out.obs_count));
  if (out.match_count > 0) {
    out.iou_match_mean = best.sum / static_cast<double>(out.match_count);
  }
  return out;
}
}  // namespace

std::string SpVisionTracker::WriteSolverConfig(const SolverConfig & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "solver_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "camera_matrix: " << VecToYaml(cfg.camera_matrix) << "\n";
  ofs << "distort_coeffs: " << VecToYaml(cfg.distort_coeffs) << "\n";
  ofs << "R_gimbal2imubody: " << VecToYaml(cfg.R_gimbal2imubody) << "\n";
  ofs << "R_camera2gimbal: " << VecToYaml(cfg.R_camera2gimbal) << "\n";
  ofs << "t_camera2gimbal: " << VecToYaml(cfg.t_camera2gimbal) << "\n";
  return path.string();
}

std::string SpVisionTracker::WriteTrackerConfig(const TrackerConfig & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "tracker_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "enemy_color: \"" << cfg.enemy_color << "\"\n";
  ofs << "min_detect_count: " << cfg.min_detect_count << "\n";
  ofs << "max_temp_lost_count: " << cfg.max_temp_lost_count << "\n";
  ofs << "outpost_max_temp_lost_count: " << cfg.outpost_max_temp_lost_count << "\n";
  ofs << "max_match_distance: " << cfg.max_match_distance << "\n";
  ofs << "max_match_yaw_diff: " << cfg.max_match_yaw_diff << "\n";
  ofs << "outpost_extra_match_yaw_diff: " << cfg.outpost_extra_match_yaw_diff << "\n";
  return path.string();
}

void SpVisionTracker::InitOutpostCsvLog()
{
  if (!cfg_.enable_outpost_debug_log) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(cfg_.outpost_debug_log_dir, ec);
  if (ec) {
    XR_LOG_WARN(
      "[SpVisionTracker] create outpost log dir failed: %s", cfg_.outpost_debug_log_dir.c_str());
    cfg_.enable_outpost_debug_log = false;
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);

  std::ostringstream name;
  name << "outpost_debug_" << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S") << ".csv";
  outpost_csv_path_ = (std::filesystem::path(cfg_.outpost_debug_log_dir) / name.str()).string();
  outpost_csv_file_.open(outpost_csv_path_, std::ios::out | std::ios::trunc);
  if (!outpost_csv_file_.is_open()) {
    XR_LOG_WARN("[SpVisionTracker] open outpost csv log failed: %s", outpost_csv_path_.c_str());
    cfg_.enable_outpost_debug_log = false;
    return;
  }

  outpost_csv_file_
    << "seq,time_s,state,target_name,is_outpost,target_count,armors_count,match_ok,"
       "candidate_count,reject_code,match_pos_diff,match_yaw_diff,match_reproj_err,match_score,match_margin,match_id,ambiguous_reject,gate_distance,gate_yaw,pre_gate_pos_diff,pre_gate_yaw_diff,pre_gate_reproj_err,pre_gate_score,pre_gate_id,pre_gate_fail_mask,used_relaxed_jump,min_reproj_err_px,"
       "used_emergency_relock,min_center_err_px,min_center_dy_px,min_bbox_iou,set_iou_norm,set_iou_visible,set_iou_match_mean,set_coverage,set_visible_recall,set_pred_count,set_obs_count,set_match_count,obs_z_best,pred_z_best,z_err_abs_m,geom_match_ok,"
       "outpost_idx,outpost_dz,outpost_z_diff,outpost_switch,decision_match,aim_valid,fire\n";
  outpost_csv_file_.flush();
  outpost_csv_header_written_ = true;
  XR_LOG_PASS("[SpVisionTracker] outpost csv log: %s", outpost_csv_path_.c_str());
}

void SpVisionTracker::InitDebugVideoWriter(const cv::Mat & frame)
{
  if (!cfg_.save_debug_video || debug_video_ready_) {
    return;
  }
  if (frame.empty()) {
    return;
  }

  std::filesystem::path out_path(cfg_.debug_video_path);
  std::error_code ec;
  if (!out_path.parent_path().empty()) {
    std::filesystem::create_directories(out_path.parent_path(), ec);
  }

  const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  debug_video_writer_.open(
    out_path.string(), fourcc, cfg_.debug_video_fps, frame.size(), true);
  if (!debug_video_writer_.isOpened()) {
    XR_LOG_WARN(
      "[SpVisionTracker] failed to open debug video writer: %s", out_path.string().c_str());
    cfg_.save_debug_video = false;
    return;
  }

  debug_video_ready_ = true;
  debug_video_frames_ = 0;
  XR_LOG_PASS("[SpVisionTracker] debug video: %s", out_path.string().c_str());
}

void SpVisionTracker::AppendOutpostCsvLog(
  const sp_xr::ArmorFrame & armors_msg, const std::list<auto_aim::Target> & targets,
  double min_reproj_err_px, double min_center_err_px, double min_bbox_iou, double min_center_dy_px,
  double obs_z_best, double pred_z_best, double set_iou_norm, double set_iou_visible,
  double set_iou_match_mean, double set_coverage, double set_visible_recall, int set_pred_count,
  int set_obs_count, int set_match_count)
{
  if (!cfg_.enable_outpost_debug_log || !outpost_csv_file_.is_open() || !outpost_csv_header_written_) {
    return;
  }

  if (!has_outpost_t0_) {
    outpost_t0_ = armors_msg.timestamp;
    has_outpost_t0_ = true;
  }
  const double time_s = std::chrono::duration_cast<std::chrono::duration<double>>(
                          armors_msg.timestamp - outpost_t0_)
                          .count();

  const auto & dbg = tracker_.debug_info();
  const bool has_target = !targets.empty();
  const auto target_name = has_target ? targets.front().name : auto_aim::ArmorName::not_armor;
  const bool is_outpost = has_target && (target_name == auto_aim::ArmorName::outpost);

  bool decision_match = false;
  bool fire = false;
  uint8_t aim_valid = 0U;
  {
    LibXR::Mutex::LockGuard lock(aim_lock_);
    if (has_aim_command_.load()) {
      decision_match =
        (latest_aim_command_.seq == armors_msg.seq) && (latest_aim_command_.aim_point_valid != 0U);
      fire = (latest_aim_command_.seq == armors_msg.seq) && latest_aim_command_.command.shoot;
      aim_valid = latest_aim_command_.aim_point_valid;
    }
  }

  const double z_err_abs_m =
    (std::isfinite(obs_z_best) && std::isfinite(pred_z_best)) ? std::abs(obs_z_best - pred_z_best) :
                                                                 std::numeric_limits<double>::quiet_NaN();
  const bool geom_match_ok =
    dbg.matched && std::isfinite(min_reproj_err_px) && min_reproj_err_px < 8.0 &&
    std::isfinite(min_center_err_px) && min_center_err_px < 35.0 &&
    std::isfinite(min_center_dy_px) && min_center_dy_px < 22.0 &&
    std::isfinite(min_bbox_iou) && min_bbox_iou > 0.25 && std::isfinite(set_iou_visible) &&
    set_iou_visible > 0.45 && std::isfinite(set_visible_recall) && set_visible_recall > 0.60 &&
    std::isfinite(z_err_abs_m) &&
    z_err_abs_m < 0.040;

  outpost_csv_file_ << armors_msg.seq << "," << std::fixed << std::setprecision(6) << time_s << ","
                    << tracker_.state() << "," << static_cast<int>(target_name) << ","
                    << (is_outpost ? 1 : 0) << "," << targets.size() << "," << armors_msg.armors.size()
                    << "," << (dbg.matched ? 1 : 0) << "," << dbg.candidate_count << ","
                    << dbg.reject_code << "," << dbg.matched_pos_diff << ","
                    << dbg.matched_yaw_diff << "," << dbg.matched_reproj_err << ","
                    << dbg.matched_score << "," << dbg.match_margin << "," << dbg.matched_id << ","
                    << (dbg.ambiguous_reject ? 1 : 0) << ","
                    << dbg.gate_distance << "," << dbg.gate_yaw << ","
                    << dbg.pre_gate_pos_diff << "," << dbg.pre_gate_yaw_diff << ","
                    << dbg.pre_gate_reproj_err << "," << dbg.pre_gate_score << ","
                    << dbg.pre_gate_id << "," << dbg.pre_gate_fail_mask << ","
                    << (dbg.used_relaxed_jump ? 1 : 0) << "," << min_reproj_err_px << ","
                    << (dbg.used_emergency_relock ? 1 : 0) << ","
                    << min_center_err_px << "," << min_center_dy_px << "," << min_bbox_iou << ","
                    << set_iou_norm << "," << set_iou_visible << "," << set_iou_match_mean << ","
                    << set_coverage << "," << set_visible_recall << "," << set_pred_count << ","
                    << set_obs_count << "," << set_match_count << ","
                    << obs_z_best << "," << pred_z_best << "," << z_err_abs_m << ","
                    << (geom_match_ok ? 1 : 0) << ","
                    << auto_aim::Target::debug_outpost_idx() << ","
                    << auto_aim::Target::debug_outpost_dz() << ","
                    << auto_aim::Target::debug_last_outpost_z_diff() << ","
                    << (auto_aim::Target::debug_last_outpost_switch() ? 1 : 0) << ","
                    << (decision_match ? 1 : 0) << "," << static_cast<unsigned int>(aim_valid) << ","
                    << (fire ? 1 : 0) << "\n";

  ++outpost_rows_;
  if ((outpost_rows_ % 20) == 0) {
    outpost_csv_file_.flush();
  }
}

SpVisionTracker::SpVisionTracker(
  LibXR::HardwareContainer &, LibXR::ApplicationManager & app, Config cfg)
: cfg_(std::move(cfg)),
  solver_config_path_(WriteSolverConfig(cfg_.solver, this)),
  tracker_config_path_(WriteTrackerConfig(cfg_.tracker, this)),
  solver_(solver_config_path_),
  tracker_(tracker_config_path_, solver_),
  last_bad_frame_seq_(0),
  bad_frames_saved_(0),
  has_sample_frame_ts_(false),
  last_sample_frame_ts_(),
  sample_frames_saved_(0)
{
  LibXR::Topic state_topic(
    LibXR::Topic::FindOrCreate<sp_xr::GimbalState>("gimbal_state", &vision_domain_));
  auto state_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionTracker * self, LibXR::RawData & data) {
      auto * state = reinterpret_cast<sp_xr::GimbalState *>(data.addr_);
      self->OnGimbalState(*state);
    },
    this);
  state_topic.RegisterCallback(state_cb);

  LibXR::Topic armors_topic(
    LibXR::Topic::FindOrCreate<sp_xr::ArmorFrame>("armors_result", &vision_domain_));
  auto armors_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionTracker * self, LibXR::RawData & data) {
      auto * armors = reinterpret_cast<sp_xr::ArmorFrame *>(data.addr_);
      self->OnArmors(*armors);
    },
    this);
  armors_topic.RegisterCallback(armors_cb);

  LibXR::Topic image_topic(
    LibXR::Topic::FindOrCreate<sp_xr::ImageFrame>("image_raw", &vision_domain_));
  auto image_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionTracker * self, LibXR::RawData & data) {
      auto * frame = reinterpret_cast<sp_xr::ImageFrame *>(data.addr_);
      self->OnImage(*frame);
    },
    this);
  image_topic.RegisterCallback(image_cb);

  LibXR::Topic aim_topic(
    LibXR::Topic::FindOrCreate<sp_xr::AimCommand>("aim_command", &vision_domain_));
  auto aim_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionTracker * self, LibXR::RawData & data) {
      auto * command = reinterpret_cast<sp_xr::AimCommand *>(data.addr_);
      self->OnAimCommand(*command);
    },
    this);
  aim_topic.RegisterCallback(aim_cb);

  InitOutpostCsvLog();
  app.Register(*this);
  XR_LOG_PASS("[SpVisionTracker] initialized.");
}

void SpVisionTracker::OnImage(const sp_xr::ImageFrame & frame)
{
  if (!cfg_.debug_view || frame.img.empty()) {
    return;
  }

  LibXR::Mutex::LockGuard lock(image_lock_);
  latest_image_ = frame.img.clone();
  latest_image_seq_ = frame.seq;
  has_image_.store(true);
}

void SpVisionTracker::OnAimCommand(const sp_xr::AimCommand & command)
{
  LibXR::Mutex::LockGuard lock(aim_lock_);
  latest_aim_command_ = command;
  has_aim_command_.store(true);
}

void SpVisionTracker::OnGimbalState(const sp_xr::GimbalState & state)
{
  LibXR::Mutex::LockGuard lock(state_lock_);
  latest_state_ = state;
  has_state_.store(true);
}

void SpVisionTracker::OnArmors(const sp_xr::ArmorFrame & armors_msg)
{
  if (!has_state_.load()) {
    return;
  }

  sp_xr::GimbalState state;
  {
    LibXR::Mutex::LockGuard lock(state_lock_);
    state = latest_state_;
  }

  solver_.set_R_gimbal2world(Eigen::Quaterniond(
    state.q.w(), state.q.x(), state.q.y(), state.q.z()));
  auto armors = armors_msg.armors;
  auto targets = tracker_.track(armors, armors_msg.timestamp);

  sp_xr::TrackFrame output;
  output.targets = std::move(targets);
  output.timestamp = armors_msg.timestamp;
  output.seq = armors_msg.seq;

  double min_reproj_err_px = std::numeric_limits<double>::quiet_NaN();
  double min_center_err_px = std::numeric_limits<double>::quiet_NaN();
  double min_center_dy_px = std::numeric_limits<double>::quiet_NaN();
  double min_bbox_iou = std::numeric_limits<double>::quiet_NaN();
  double set_iou_norm = std::numeric_limits<double>::quiet_NaN();
  double set_iou_visible = std::numeric_limits<double>::quiet_NaN();
  double set_iou_match_mean = std::numeric_limits<double>::quiet_NaN();
  double set_coverage = std::numeric_limits<double>::quiet_NaN();
  double set_visible_recall = std::numeric_limits<double>::quiet_NaN();
  int set_pred_count = 0;
  int set_obs_count = 0;
  int set_match_count = 0;
  double obs_z_best = std::numeric_limits<double>::quiet_NaN();
  double pred_z_best = std::numeric_limits<double>::quiet_NaN();
  if (!output.targets.empty()) {
    const auto & target = output.targets.front();
    const auto pred_xyza = target.armor_xyza_list();
    double best_err = std::numeric_limits<double>::infinity();
    double best_center = std::numeric_limits<double>::infinity();
    double best_center_dy = std::numeric_limits<double>::infinity();
    double best_iou = 0.0;
    double best_obs_z = std::numeric_limits<double>::quiet_NaN();
    double best_pred_z = std::numeric_limits<double>::quiet_NaN();

    auto is_eval_candidate = [&](const auto_aim::Armor & armor, bool strict_name) {
      if (armor.type != target.armor_type) {
        return false;
      }
      if (!strict_name) {
        return true;
      }
      if (armor.name == target.name) {
        return true;
      }
      if (
        (target.name == auto_aim::ArmorName::outpost && armor.name == auto_aim::ArmorName::base) ||
        (target.name == auto_aim::ArmorName::base && armor.name == auto_aim::ArmorName::outpost)) {
        return true;
      }
      return false;
    };

    std::vector<std::vector<cv::Point2f>> pred_quads;
    for (const auto & pred : pred_xyza) {
      auto points = solver_.reproject_armor(pred.head(3), pred[3], target.armor_type, target.name);
      if (points.size() == 4) {
        pred_quads.push_back(points);
      }
    }

    std::vector<std::vector<cv::Point2f>> obs_quads;
    auto collect_obs = [&](bool strict_name) {
      for (const auto & armor : armors) {
        if (!is_eval_candidate(armor, strict_name)) {
          continue;
        }
        if (armor.points.size() == 4) {
          obs_quads.push_back(armor.points);
        }
      }
    };
    collect_obs(true);
    if (obs_quads.empty()) {
      collect_obs(false);
    }
    const auto set_metrics = EvaluateSetOverlap(pred_quads, obs_quads);
    set_iou_norm = set_metrics.iou_norm;
    set_iou_visible = set_metrics.iou_visible;
    set_iou_match_mean = set_metrics.iou_match_mean;
    set_coverage = set_metrics.coverage;
    set_visible_recall = set_metrics.visible_recall;
    set_pred_count = set_metrics.pred_count;
    set_obs_count = set_metrics.obs_count;
    set_match_count = set_metrics.match_count;

    auto eval_once = [&](bool strict_name) {
      bool found = false;
      for (const auto & pred : pred_xyza) {
        const auto points =
          solver_.reproject_armor(pred.head(3), pred[3], target.armor_type, target.name);
        if (points.size() != 4) {
          continue;
        }
        const cv::Point2f pred_center = QuadCenter(points);
        const cv::Rect2f pred_bbox = QuadBBox(points);
        for (const auto & armor : armors) {
          if (!is_eval_candidate(armor, strict_name)) {
            continue;
          }
          const double err = ArmorReprojectionError(armor.points, points);
          if (!std::isfinite(err)) {
            continue;
          }
          found = true;
          if (err < best_err) {
            best_err = err;
            best_center = cv::norm(armor.center - pred_center);
            best_center_dy = std::abs(static_cast<double>(armor.center.y - pred_center.y));
            best_iou = RectIou(QuadBBox(armor.points), pred_bbox);
            best_obs_z = armor.xyz_in_world[2];
            best_pred_z = pred[2];
          }
        }
      }
      return found;
    };

    bool found = eval_once(true);
    if (!found) {
      (void)eval_once(false);
    }

    if (std::isfinite(best_err)) {
      min_reproj_err_px = best_err;
      min_center_err_px = best_center;
      min_center_dy_px = best_center_dy;
      min_bbox_iou = best_iou;
      obs_z_best = best_obs_z;
      pred_z_best = best_pred_z;
    }
  }

  if (armors_msg.seq % 60 == 0) {
    XR_LOG_DEBUG(
      "[SpVisionTracker] seq=%llu armors=%zu targets=%zu", static_cast<unsigned long long>(armors_msg.seq),
      armors.size(), output.targets.size());
  }

  track_topic_.Publish(output);
  AppendOutpostCsvLog(
    armors_msg, output.targets, min_reproj_err_px, min_center_err_px, min_bbox_iou,
    min_center_dy_px, obs_z_best, pred_z_best, set_iou_norm, set_iou_visible, set_iou_match_mean,
    set_coverage, set_visible_recall, set_pred_count, set_obs_count, set_match_count);
  ShowReprojection(armors_msg, output.targets, armors);
}

void SpVisionTracker::ShowReprojection(
  const sp_xr::ArmorFrame & armors_msg, const std::list<auto_aim::Target> & targets,
  const std::list<auto_aim::Armor> & solved_armors)
{
  if ((!cfg_.debug_view && !cfg_.save_debug_video) || !has_image_.load()) {
    return;
  }

  cv::Mat canvas;
  uint64_t image_seq = 0;
  {
    LibXR::Mutex::LockGuard lock(image_lock_);
    if (latest_image_.empty()) {
      return;
    }
    canvas = latest_image_.clone();
    image_seq = latest_image_seq_;
  }
  const auto & dbg = tracker_.debug_info();
  constexpr bool kMinimalOverlay = false;
  constexpr bool kAlwaysShowAllOutpostPreds = true;

  struct OutpostPredVis
  {
    std::vector<cv::Point2f> points;
    cv::Point2f center;
    std::string level;
    double z = 0.0;
    Eigen::Vector4d xyza{0, 0, 0, 0};
    int pred_id = -1;
  };
  std::vector<OutpostPredVis> outpost_preds;
  bool outpost_show_all_hyp = false;

  if (!kMinimalOverlay) {
    cv::Mat hud = canvas.clone();
    const int hud_w = std::min(900, std::max(320, canvas.cols - 20));
    const int hud_h = std::min(250, std::max(180, canvas.rows / 3));
    cv::rectangle(hud, cv::Rect(6, 6, hud_w, hud_h), cv::Scalar(15, 15, 15), cv::FILLED);
    cv::addWeighted(hud, 0.38, canvas, 0.62, 0.0, canvas);
  }

  if (!kMinimalOverlay) {
    // Green: detector output armors.
    for (const auto & armor : solved_armors) {
      tools::draw_points(canvas, armor.points, {0, 255, 0}, 2);
      std::ostringstream det_label;
      det_label << "det " << auto_aim::ARMOR_NAMES[armor.name];
      if (armor.name == auto_aim::ArmorName::outpost) {
        det_label << " z=" << std::fixed << std::setprecision(3) << armor.xyz_in_world[2];
      }
      tools::draw_text(
        canvas, det_label.str(),
        cv::Point(static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)), {0, 255, 0}, 0.5,
        1);
    }
  }

  // Red/Orange: tracker reprojection result.
  int target_idx = 0;
  for (const auto & target : targets) {
    const cv::Scalar color = (target_idx == 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 165, 255);
    auto xyza_list = target.armor_xyza_list();

    if (target.name == auto_aim::ArmorName::outpost && target_idx == 0) {
      struct PredCand
      {
        std::vector<cv::Point2f> points;
        Eigen::Vector4d xyza{0, 0, 0, 0};
        int id = -1;
        double min_err = std::numeric_limits<double>::infinity();
      };
      std::vector<std::vector<cv::Point2f>> obs_quads;
      for (const auto & armor : solved_armors) {
        if (
          armor.name == auto_aim::ArmorName::outpost || armor.name == auto_aim::ArmorName::base ||
          armor.name == auto_aim::ArmorName::not_armor) {
          if (armor.points.size() == 4) {
            obs_quads.push_back(armor.points);
          }
        }
      }

      std::vector<PredCand> preds;
      preds.reserve(xyza_list.size());
      for (size_t i = 0; i < xyza_list.size(); ++i) {
        const auto & xyza = xyza_list[i];
        auto points = solver_.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        if (points.size() != 4) {
          continue;
        }
        double min_err = std::numeric_limits<double>::infinity();
        for (const auto & obs : obs_quads) {
          min_err = std::min(min_err, ArmorReprojectionError(obs, points));
        }
        preds.push_back(PredCand{points, xyza, static_cast<int>(i), min_err});
      }

      std::vector<int> order(static_cast<int>(preds.size()));
      for (size_t i = 0; i < preds.size(); ++i) {
        order[i] = static_cast<int>(i);
      }
      std::sort(order.begin(), order.end(), [&](int a, int b) { return preds[a].min_err < preds[b].min_err; });
      auto draw_pred = [&](const PredCand & cand, bool selected) {
        const cv::Scalar pred_color =
          selected ? cv::Scalar(255, 0, 255) : (kMinimalOverlay ? cv::Scalar(255, 170, 0) : color);
        const int thickness = selected ? 3 : 2;
        tools::draw_points(canvas, cand.points, pred_color, thickness);
        const cv::Point2f c = QuadCenter(cand.points);
        outpost_preds.push_back(OutpostPredVis{cand.points, c, "", cand.xyza[2], cand.xyza, cand.id});
      };

      int selected_idx = -1;
      for (size_t i = 0; i < preds.size(); ++i) {
        if (preds[i].id == dbg.matched_id) {
          selected_idx = static_cast<int>(i);
          break;
        }
      }
      const bool show_all_hyp =
        !dbg.matched || dbg.reject_code != 0 ||
        (std::isfinite(dbg.matched_reproj_err) &&
         dbg.matched_reproj_err > cfg_.outpost_bad_match_reproj_threshold);
      outpost_show_all_hyp = (!kMinimalOverlay) && (show_all_hyp || kAlwaysShowAllOutpostPreds);

      if (kMinimalOverlay) {
        int fallback_best = order.empty() ? -1 : order.front();
        for (size_t i = 0; i < preds.size(); ++i) {
          const bool selected =
            (static_cast<int>(i) == selected_idx) || (selected_idx < 0 && static_cast<int>(i) == fallback_best);
          draw_pred(preds[i], selected);
        }
      } else {
        if (selected_idx >= 0) {
          draw_pred(preds[static_cast<size_t>(selected_idx)], true);
        } else if (!order.empty()) {
          draw_pred(preds[static_cast<size_t>(order.front())], false);
        }
      }

      if (outpost_show_all_hyp) {
        for (int ord : order) {
          if (ord == selected_idx) {
            continue;
          }
          const auto & cand = preds[static_cast<size_t>(ord)];
          draw_pred(cand, false);
          const cv::Point2f c = QuadCenter(cand.points);
          std::ostringstream hyp;
          hyp << "hyp" << cand.id;
          tools::draw_text(
            canvas, hyp.str(), cv::Point(static_cast<int>(c.x) - 10, static_cast<int>(c.y) - 10),
            cv::Scalar(120, 120, 220), 0.40, 1);
        }
      } else if (!kMinimalOverlay) {
        int hint_count = 0;
        for (int ord : order) {
          if (ord == selected_idx || hint_count >= 2) {
            continue;
          }
          const auto & cand = preds[static_cast<size_t>(ord)];
          const cv::Point2f c = QuadCenter(cand.points);
          cv::circle(canvas, c, 3, cv::Scalar(140, 140, 210), cv::FILLED, cv::LINE_AA);
          hint_count++;
        }
      }

      // Visualize outpost rotation axis and radius rays for model diagnostics.
      if (!kMinimalOverlay && !preds.empty()) {
        const auto ekf_x = target.ekf_x();
        if (ekf_x.size() >= 9) {
          std::vector<cv::Point3f> axis_world{
            cv::Point3f(
              static_cast<float>(ekf_x[0]), static_cast<float>(ekf_x[2]), static_cast<float>(ekf_x[4]))};
          const auto axis_px = solver_.world2pixel(axis_world);
          if (!axis_px.empty()) {
            const cv::Point2f axis_c = axis_px.front();
            cv::circle(canvas, axis_c, 4, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);
            for (const auto & cand : preds) {
              cv::line(canvas, axis_c, QuadCenter(cand.points), cv::Scalar(80, 220, 220), 1, cv::LINE_AA);
            }
            std::ostringstream axis_line;
            axis_line << "axis r=" << std::fixed << std::setprecision(3) << ekf_x[8];
            tools::draw_text(
              canvas, axis_line.str(),
              cv::Point(static_cast<int>(axis_c.x) + 8, static_cast<int>(axis_c.y) - 8),
              cv::Scalar(120, 220, 220), 0.45, 1);
          }
        }
      }
    } else {
      for (size_t i = 0; i < xyza_list.size(); ++i) {
        const auto & xyza = xyza_list[i];
        auto points = solver_.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        if (points.size() != 4) {
          continue;
        }
        const bool selected = (target_idx == 0 && static_cast<int>(i) == dbg.matched_id);
        const cv::Scalar pred_color = selected ? cv::Scalar(255, 0, 255) : color;
        const int thickness = selected ? 3 : 2;
        tools::draw_points(canvas, points, pred_color, thickness);
        if (target.name == auto_aim::ArmorName::outpost) {
          const cv::Point2f c = QuadCenter(points);
          outpost_preds.push_back(OutpostPredVis{points, c, "", xyza[2], xyza, static_cast<int>(i)});
        }
      }
    }
    target_idx++;
  }

  if (outpost_preds.size() >= 3) {
    std::vector<int> order(static_cast<int>(outpost_preds.size()));
    for (size_t i = 0; i < outpost_preds.size(); ++i) {
      order[i] = static_cast<int>(i);
    }
    // Logical layering: low/mid/high comes from world z, not image y.
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return outpost_preds[a].z < outpost_preds[b].z;
    });
    outpost_preds[order[0]].level = "L";
    outpost_preds[order[1]].level = "M";
    outpost_preds[order[2]].level = "H";
  }
  for (const auto & pred : outpost_preds) {
    const bool selected = (pred.pred_id == dbg.matched_id);
    if (!selected && !outpost_show_all_hyp) {
      continue;
    }
    if (kMinimalOverlay) {
      continue;
    }
    std::ostringstream lvl;
    lvl << "pred" << pred.pred_id << ":" << pred.level << (selected ? "*" : "") << " z="
        << std::fixed << std::setprecision(3) << pred.z;
    tools::draw_text(
      canvas, lvl.str(), cv::Point(static_cast<int>(pred.center.x) + 6, static_cast<int>(pred.center.y) - 6),
      selected ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 200, 255), 0.45, 1);
  }

  sp_xr::AimCommand aim_command{};
  bool has_aim_command = false;
  {
    LibXR::Mutex::LockGuard lock(aim_lock_);
    has_aim_command = has_aim_command_.load();
    if (has_aim_command) {
      aim_command = latest_aim_command_;
    }
  }

  if (!kMinimalOverlay) {
    std::ostringstream title;
    title << "reprojection det_seq=" << armors_msg.seq << " img_seq=" << image_seq
          << " det=" << armors_msg.armors.size() << " tar=" << targets.size();
    tools::draw_text(canvas, title.str(), {10, 28}, {255, 255, 255}, 0.7, 2);
  }
  if (!targets.empty() && targets.front().name == auto_aim::ArmorName::outpost) {
    if (!kMinimalOverlay) {
      std::ostringstream outpost_line;
      outpost_line << "outpost idx=" << auto_aim::Target::debug_outpost_idx() << " dz=" << std::fixed
                   << std::setprecision(3) << auto_aim::Target::debug_outpost_dz() << " zdiff="
                   << auto_aim::Target::debug_last_outpost_z_diff();
      tools::draw_text(canvas, outpost_line.str(), {10, 112}, {0, 200, 255}, 0.6, 2);

      std::ostringstream trk_line;
      trk_line << "trk m=" << (dbg.matched ? 1 : 0) << " rc=" << dbg.reject_code << " id=" << dbg.matched_id
               << " rp=" << std::fixed << std::setprecision(1) << dbg.matched_reproj_err
               << " pre=" << dbg.pre_gate_reproj_err << " relax=" << (dbg.used_relaxed_jump ? 1 : 0)
               << " emg=" << (dbg.used_emergency_relock ? 1 : 0);
      tools::draw_text(canvas, trk_line.str(), {10, 140}, {200, 220, 255}, 0.55, 2);
    }

    for (const auto & armor : solved_armors) {
      if (armor.name != auto_aim::ArmorName::outpost || outpost_preds.empty()) {
        continue;
      }
      double best_err = std::numeric_limits<double>::infinity();
      const OutpostPredVis * best_pred = nullptr;
      for (const auto & pred : outpost_preds) {
        const double err = ArmorReprojectionError(armor.points, pred.points);
        if (err < best_err) {
          best_err = err;
          best_pred = &pred;
        }
      }
      if (best_pred != nullptr && std::isfinite(best_err)) {
        const double center_err = cv::norm(armor.center - best_pred->center);
        const double z_err = std::abs(armor.xyz_in_world[2] - best_pred->z);
        const cv::Scalar line_color = (best_err > 12.0 || center_err > 35.0) ? cv::Scalar(0, 0, 255) :
                                                                             cv::Scalar(0, 220, 255);
        if (!kMinimalOverlay) {
          cv::line(canvas, armor.center, best_pred->center, line_color, 1, cv::LINE_AA);
          std::ostringstream match_line;
          match_line << "obs->pred" << best_pred->level << " err=" << std::fixed << std::setprecision(1)
                     << best_err << "px c=" << center_err << "px"
                     << " obs_z=" << std::setprecision(3) << armor.xyz_in_world[2]
                     << " pred_z=" << best_pred->z << " dz=" << z_err;
          tools::draw_text(
            canvas, match_line.str(),
            cv::Point(static_cast<int>(armor.center.x) + 6, static_cast<int>(armor.center.y) + 18),
            line_color, 0.5, 1);
        }
      }
    }

    const OutpostPredVis * selected_pred = nullptr;
    for (const auto & pred : outpost_preds) {
      if (pred.pred_id == dbg.matched_id) {
        selected_pred = &pred;
        break;
      }
    }
    if (selected_pred == nullptr && !outpost_preds.empty()) {
      selected_pred = &outpost_preds.front();
    }
    if (selected_pred != nullptr) {
      const auto_aim::Armor * selected_obs = nullptr;
      double best_err = std::numeric_limits<double>::infinity();
      for (const auto & armor : solved_armors) {
        const bool maybe_outpost =
          armor.name == auto_aim::ArmorName::outpost || armor.name == auto_aim::ArmorName::base ||
          armor.name == auto_aim::ArmorName::not_armor;
        if (!maybe_outpost || armor.points.size() != 4) {
          continue;
        }
        const double err = ArmorReprojectionError(armor.points, selected_pred->points);
        if (err < best_err) {
          best_err = err;
          selected_obs = &armor;
        }
      }
      if (selected_obs != nullptr && std::isfinite(best_err)) {
        const Eigen::Vector3d dxyz = selected_obs->xyz_in_world - selected_pred->xyza.head(3);
        const double dyaw = AbsAngleDiff(selected_obs->ypr_in_world[0], selected_pred->xyza[3]);
        if (!kMinimalOverlay) {
          std::ostringstream obs_line;
          obs_line << "obs xyz=(" << std::fixed << std::setprecision(3) << selected_obs->xyz_in_world[0]
                   << "," << selected_obs->xyz_in_world[1] << "," << selected_obs->xyz_in_world[2]
                   << ") yaw=" << selected_obs->ypr_in_world[0];
          tools::draw_text(canvas, obs_line.str(), {10, 168}, {120, 255, 120}, 0.52, 2);

          std::ostringstream pred_line;
          pred_line << "pred id=" << selected_pred->pred_id << " xyz=(" << std::fixed
                    << std::setprecision(3) << selected_pred->xyza[0] << "," << selected_pred->xyza[1]
                    << "," << selected_pred->xyza[2] << ") yaw=" << selected_pred->xyza[3];
          tools::draw_text(canvas, pred_line.str(), {10, 194}, {0, 210, 255}, 0.52, 2);

          std::ostringstream res_line;
          res_line << "res dxyz=(" << std::fixed << std::setprecision(3) << dxyz[0] << ","
                   << dxyz[1] << "," << dxyz[2] << ") dyaw=" << dyaw << " reproj=" << best_err;
          const cv::Scalar res_color =
            (best_err > cfg_.outpost_bad_match_reproj_threshold) ? cv::Scalar(0, 0, 255) :
                                                                    cv::Scalar(255, 255, 255);
          tools::draw_text(canvas, res_line.str(), {10, 220}, res_color, 0.52, 2);
        }
      }
    }
  }

  if (has_aim_command) {
    if (!kMinimalOverlay) {
      std::ostringstream aim_line;
      aim_line << "cmd seq=" << aim_command.seq << " ctrl=" << (aim_command.command.control ? 1 : 0)
               << " fire=" << (aim_command.command.shoot ? 1 : 0) << " yaw=" << std::fixed
               << std::setprecision(3) << aim_command.command.yaw << " pitch=" << aim_command.command.pitch;
      const cv::Scalar cmd_color =
        aim_command.command.shoot ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0);
      tools::draw_text(canvas, aim_line.str(), {10, 56}, cmd_color, 0.7, 2);
    }
    const bool decision_match = (aim_command.seq == armors_msg.seq) && (aim_command.aim_point_valid != 0U);
    if (armors_msg.seq % 60 == 0) {
      XR_LOG_DEBUG(
        "[SpVisionTracker] decision_match=%d det_seq=%llu cmd_seq=%llu aim_valid=%u",
        decision_match ? 1 : 0, static_cast<unsigned long long>(armors_msg.seq),
        static_cast<unsigned long long>(aim_command.seq), static_cast<unsigned int>(aim_command.aim_point_valid));
    }
    if (decision_match) {
      Eigen::Vector3d final_xyz(
        static_cast<double>(aim_command.aim_x), static_cast<double>(aim_command.aim_y),
        static_cast<double>(aim_command.aim_z));
      auto final_points = solver_.reproject_armor(
        final_xyz, static_cast<double>(aim_command.aim_a),
        static_cast<auto_aim::ArmorType>(aim_command.aim_armor_type),
        static_cast<auto_aim::ArmorName>(aim_command.aim_armor_name));
      if (final_points.size() == 4) {
        cv::Point2f center(0.0f, 0.0f);
        for (const auto & p : final_points) {
          center += p;
        }
        center *= 0.25f;
        const cv::Point img_center(canvas.cols / 2, canvas.rows / 2);
        const cv::Point impact_pt(static_cast<int>(center.x), static_cast<int>(center.y));
        if (!kMinimalOverlay) {
          cv::line(canvas, img_center, impact_pt, {0, 180, 255}, 1, cv::LINE_AA);
        }
        cv::drawMarker(canvas, impact_pt, {0, 255, 255}, cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
        if (!kMinimalOverlay) {
          tools::draw_text(
            canvas, "FINAL_TARGET",
            cv::Point(static_cast<int>(center.x) + 8, static_cast<int>(center.y) - 8), {0, 255, 255},
            0.7, 2);
          tools::draw_text(
            canvas, "IMPACT_PRED",
            cv::Point(static_cast<int>(center.x) + 8, static_cast<int>(center.y) + 14), {0, 255, 255},
            0.55, 2);
        }
        if (aim_command.command.shoot && !kMinimalOverlay) {
          cv::circle(canvas, center, 10, {0, 0, 255}, 2);
          tools::draw_text(canvas, "FIRE", {10, 84}, {0, 0, 255}, 1.0, 3);
        }
      }
    } else if (!kMinimalOverlay) {
      tools::draw_text(canvas, "FINAL_TARGET pending", {10, 84}, {0, 255, 255}, 0.7, 2);
    }
  }

  if (cfg_.save_debug_video) {
    InitDebugVideoWriter(canvas);
    if (debug_video_ready_) {
      debug_video_writer_.write(canvas);
      debug_video_frames_++;
      if ((debug_video_frames_ % 120) == 0) {
        XR_LOG_DEBUG(
          "[SpVisionTracker] debug video frames=%llu",
          static_cast<unsigned long long>(debug_video_frames_));
      }
    }
  }

  if (!targets.empty() && targets.front().name == auto_aim::ArmorName::outpost &&
      cfg_.save_outpost_bad_frames) {
    const bool has_det = !solved_armors.empty();
    const bool severe =
      (has_det && dbg.reject_code == 2) ||
      (!dbg.matched && std::isfinite(dbg.pre_gate_reproj_err) &&
       dbg.pre_gate_reproj_err > cfg_.outpost_bad_pre_reproj_threshold) ||
      (dbg.matched && std::isfinite(dbg.matched_reproj_err) &&
       dbg.matched_reproj_err > cfg_.outpost_bad_match_reproj_threshold);
    const uint64_t min_interval = static_cast<uint64_t>(std::max(1, cfg_.outpost_bad_min_interval));
    if (severe && (bad_frames_saved_ == 0 || armors_msg.seq > last_bad_frame_seq_ + min_interval)) {
      std::error_code ec;
      std::filesystem::create_directories(cfg_.outpost_bad_frame_dir, ec);
      if (!ec) {
        std::ostringstream name;
        name << "seq_" << armors_msg.seq << "_rc" << dbg.reject_code << "_m"
             << (dbg.matched ? 1 : 0) << "_pre" << std::fixed << std::setprecision(1)
             << dbg.pre_gate_reproj_err << ".png";
        const auto path = (std::filesystem::path(cfg_.outpost_bad_frame_dir) / name.str()).string();
        if (cv::imwrite(path, canvas)) {
          last_bad_frame_seq_ = armors_msg.seq;
          bad_frames_saved_++;
          if ((bad_frames_saved_ % 10) == 0) {
            XR_LOG_DEBUG(
              "[SpVisionTracker] saved outpost bad frames=%llu",
              static_cast<unsigned long long>(bad_frames_saved_));
          }
        }
      }
    }
  }

  if (cfg_.save_outpost_sample_frames) {
    bool has_outpost_context =
      (!targets.empty() && targets.front().name == auto_aim::ArmorName::outpost);
    if (!has_outpost_context) {
      for (const auto & armor : solved_armors) {
        if (armor.name == auto_aim::ArmorName::outpost || armor.name == auto_aim::ArmorName::base ||
            armor.name == auto_aim::ArmorName::not_armor) {
          has_outpost_context = true;
          break;
        }
      }
    }

    if (has_outpost_context) {
      const auto interval_ms = std::max(1, cfg_.outpost_sample_interval_ms);
      const bool due = !has_sample_frame_ts_ ||
                       (std::chrono::duration_cast<std::chrono::milliseconds>(
                          armors_msg.timestamp - last_sample_frame_ts_)
                          .count() >= interval_ms);
      if (due) {
        std::error_code ec;
        std::filesystem::create_directories(cfg_.outpost_sample_frame_dir, ec);
        if (!ec) {
          std::ostringstream name;
          name << "seq_" << armors_msg.seq << "_rc" << dbg.reject_code << "_m" << (dbg.matched ? 1 : 0)
               << "_pre" << std::fixed << std::setprecision(1) << dbg.pre_gate_reproj_err << ".png";
          const auto path =
            (std::filesystem::path(cfg_.outpost_sample_frame_dir) / name.str()).string();
          if (cv::imwrite(path, canvas)) {
            has_sample_frame_ts_ = true;
            last_sample_frame_ts_ = armors_msg.timestamp;
            sample_frames_saved_++;
            if ((sample_frames_saved_ % 50) == 0) {
              XR_LOG_DEBUG(
                "[SpVisionTracker] saved 100ms frames=%llu",
                static_cast<unsigned long long>(sample_frames_saved_));
            }
          }
        }
      }
    }
  }

  if (cfg_.debug_view) {
    cv::imshow("reprojection", canvas);
    cv::waitKey(1);
  }
}
