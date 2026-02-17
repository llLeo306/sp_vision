#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  max_match_distance_ = yaml["max_match_distance"] ? yaml["max_match_distance"].as<double>() : 0.15;
  max_match_yaw_diff_ = yaml["max_match_yaw_diff"] ? yaml["max_match_yaw_diff"].as<double>() : 1.0;
  outpost_extra_match_yaw_diff_ =
    yaml["outpost_extra_match_yaw_diff"] ? yaml["outpost_extra_match_yaw_diff"].as<double>() : 0.7;
  outpost_min_match_margin_ =
    yaml["outpost_min_match_margin"] ? yaml["outpost_min_match_margin"].as<double>() : 0.03;
  outpost_ambiguity_reproj_floor_ = yaml["outpost_ambiguity_reproj_floor"]
                                      ? yaml["outpost_ambiguity_reproj_floor"].as<double>()
                                      : 1.0;
  outpost_top_relock_streak_ = 0;
  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }
const Tracker::DebugInfo & Tracker::debug_info() const { return debug_info_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  if (target_.name == ArmorName::outpost) {
    if (!found && debug_info_.reject_code == 2) {
      outpost_top_relock_streak_++;
    } else {
      outpost_top_relock_streak_ = 0;
    }
  } else {
    outpost_top_relock_streak_ = 0;
  }

  constexpr bool kEnableTopLevelOutpostRelock = true;
  if (
    kEnableTopLevelOutpostRelock &&
    !found && target_.name == ArmorName::outpost && debug_info_.reject_code == 2 &&
    outpost_top_relock_streak_ >= 2 && !armors.empty()) {
    Armor const * best = nullptr;
    for (const auto & armor : armors) {
      if (armor.name == ArmorName::outpost) {
        best = &armor;
        break;
      }
      if (best == nullptr && armor.name == ArmorName::base) {
        best = &armor;
      }
      if (best == nullptr && armor.name == ArmorName::not_armor) {
        best = &armor;
      }
    }
    if (best == nullptr && !armors.empty()) {
      best = &armors.front();
    }
    if (best != nullptr) {
      Armor relock_armor = *best;
      relock_armor.name = ArmorName::outpost;
      relock_armor.type = ArmorType::small;
      solver_.solve(relock_armor);
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
      target_ = Target(relock_armor, t, 0.275, 3, P0_dig);
      target_.armor_type = ArmorType::small;
      target_.update(relock_armor, true);
      found = true;
      debug_info_.matched = true;
      debug_info_.reject_code = 0;
      debug_info_.used_emergency_relock = true;
      debug_info_.matched_id = target_.last_id;
      debug_info_.matched_reproj_err = 0.0;
      debug_info_.matched_pos_diff = 0.0;
      debug_info_.matched_yaw_diff = 0.0;
      outpost_top_relock_streak_ = 0;
    }
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  if (target_.name == ArmorName::outpost) {
    if (!found && debug_info_.reject_code == 2) {
      outpost_top_relock_streak_++;
    } else {
      outpost_top_relock_streak_ = 0;
    }
  } else {
    outpost_top_relock_streak_ = 0;
  }

  constexpr bool kEnableTopLevelOutpostRelock = true;
  if (
    kEnableTopLevelOutpostRelock &&
    !found && target_.name == ArmorName::outpost && debug_info_.reject_code == 2 &&
    outpost_top_relock_streak_ >= 2 && !armors.empty()) {
    Armor const * best = nullptr;
    for (const auto & armor : armors) {
      if (armor.name == ArmorName::outpost) {
        best = &armor;
        break;
      }
      if (best == nullptr && armor.name == ArmorName::base) {
        best = &armor;
      }
      if (best == nullptr && armor.name == ArmorName::not_armor) {
        best = &armor;
      }
    }
    if (best == nullptr && !armors.empty()) {
      best = &armors.front();
    }
    if (best != nullptr) {
      Armor relock_armor = *best;
      relock_armor.name = ArmorName::outpost;
      relock_armor.type = ArmorType::small;
      solver_.solve(relock_armor);
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
      target_ = Target(relock_armor, t, 0.275, 3, P0_dig);
      target_.armor_type = ArmorType::small;
      target_.update(relock_armor, true);
      found = true;
      debug_info_.matched = true;
      debug_info_.reject_code = 0;
      debug_info_.used_emergency_relock = true;
      debug_info_.matched_id = target_.last_id;
      debug_info_.matched_reproj_err = 0.0;
      debug_info_.matched_pos_diff = 0.0;
      debug_info_.matched_yaw_diff = 0.0;
      outpost_top_relock_streak_ = 0;
    }
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    const int min_detect_need = (target_.name == ArmorName::outpost) ? 1 : min_detect_count_;
    if (min_detect_need <= 1) {
      state_ = "tracking";
      detect_count_ = min_detect_need;
    } else {
      state_ = "detecting";
      detect_count_ = 1;
    }
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      const int min_detect_need =
        (target_.name == ArmorName::outpost) ? std::max(2, min_detect_count_ / 2) : min_detect_count_;
      if (detect_count_ >= min_detect_need) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto armor = armors.front();
  if (armor.name == ArmorName::outpost) {
    armor.type = ArmorType::small;
  }
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.275, 3, P0_dig);
    target_.armor_type = ArmorType::small;
    auto reprojection_error = [](const std::vector<cv::Point2f> & obs,
                                 const std::vector<cv::Point2f> & pred) {
      if (obs.size() != 4 || pred.size() != 4) {
        return std::numeric_limits<double>::infinity();
      }
      double best = std::numeric_limits<double>::infinity();
      for (int shift = 0; shift < 4; ++shift) {
        double err_forward = 0.0;
        double err_reverse = 0.0;
        for (int i = 0; i < 4; ++i) {
          const int j_forward = (i + shift) % 4;
          const int j_reverse = (shift - i + 8) % 4;
          err_forward +=
            cv::norm(obs[static_cast<size_t>(i)] - pred[static_cast<size_t>(j_forward)]);
          err_reverse +=
            cv::norm(obs[static_cast<size_t>(i)] - pred[static_cast<size_t>(j_reverse)]);
        }
        best = std::min(best, std::min(err_forward, err_reverse));
      }
      return best;
    };
    int init_id = 0;
    double best_err = std::numeric_limits<double>::infinity();
    const auto xyza = target_.armor_xyza_list();
    for (size_t i = 0; i < xyza.size(); ++i) {
      const auto reproj =
        solver_.reproject_armor(xyza[i].head(3), xyza[i][3], ArmorType::small, ArmorName::outpost);
      const double err = reprojection_error(armor.points, reproj);
      if (err < best_err) {
        best_err = err;
        init_id = static_cast<int>(i);
      }
    }
    target_.update(armor, true, init_id);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);
  debug_info_ = {};

  // Keep the original path untouched for non-outpost targets to avoid affecting 4-armor behavior.
  if (target_.name != ArmorName::outpost) {
    int found_count = 0;
    double min_x = 1e10;  // 画面最左侧
    for (const auto & armor : armors) {
      if (armor.name != target_.name || armor.type != target_.armor_type) continue;
      found_count++;
      min_x = armor.center.x < min_x ? armor.center.x : min_x;
    }
    debug_info_.candidate_count = found_count;

    if (found_count == 0) {
      debug_info_.reject_code = 1;
      return false;
    }

    bool updated = false;
    for (auto & armor : armors) {
      if (
        armor.name != target_.name || armor.type != target_.armor_type
        //  || armor.center.x != min_x
      )
        continue;

      solver_.solve(armor);

      target_.update(armor);
      updated = true;
    }

    if (!updated) {
      debug_info_.reject_code = 2;
      return false;
    }

    debug_info_.matched = true;
    debug_info_.reject_code = 0;
    return true;
  }

  target_.armor_type = ArmorType::small;
  const auto pred_xyza = target_.armor_xyza_list();
  if (pred_xyza.empty()) {
    debug_info_.reject_code = 2;
    return false;
  }

  auto reprojection_error = [](const std::vector<cv::Point2f> & obs,
                               const std::vector<cv::Point2f> & pred) {
    if (obs.size() != 4 || pred.size() != 4) {
      return std::numeric_limits<double>::infinity();
    }
    // Robust to detector corner order jitter: test cyclic shifts and reversed order.
    double best = std::numeric_limits<double>::infinity();
    for (int shift = 0; shift < 4; ++shift) {
      double err_forward = 0.0;
      double err_reverse = 0.0;
      for (int i = 0; i < 4; ++i) {
        const int j_forward = (i + shift) % 4;
        const int j_reverse = (shift - i + 8) % 4;
        err_forward += cv::norm(obs[static_cast<size_t>(i)] - pred[static_cast<size_t>(j_forward)]);
        err_reverse += cv::norm(obs[static_cast<size_t>(i)] - pred[static_cast<size_t>(j_reverse)]);
      }
      best = std::min(best, std::min(err_forward, err_reverse));
    }
    return best;
  };

  struct MatchCandidate
  {
    Armor * armor = nullptr;
    bool relaxed = false;
    bool type_compatible = true;
    int id = -1;
    double pos_diff = std::numeric_limits<double>::infinity();
    double yaw_diff = std::numeric_limits<double>::infinity();
    double z_diff = std::numeric_limits<double>::infinity();
    double reproj_err = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity();
    int fail_mask = 8;  // bit0=pos bit1=yaw bit2=geom bit3=invalid_id
  };

  const double pos_gate = std::max(max_match_distance_ * 1.8, 0.26);
  const double yaw_gate = max_match_yaw_diff_ + outpost_extra_match_yaw_diff_ + 0.35;
  const double reproj_gate = 23.0;
  const double z_gate = 0.085;

  auto evaluate_candidates = [&](bool relaxed_name) {
    std::vector<MatchCandidate> out;
    out.reserve(armors.size());
    for (auto & armor : armors) {
      const bool strict_match = (armor.name == target_.name);
      if (!relaxed_name && !strict_match) {
        continue;
      }

      const auto original_type = armor.type;
      const bool type_compatible = (original_type == ArmorType::small);
      if (!type_compatible) {
        armor.type = ArmorType::small;
      }
      solver_.solve(armor);
      if (!type_compatible) {
        armor.type = original_type;
      }

      MatchCandidate best;
      best.armor = &armor;
      best.relaxed = relaxed_name;
      best.type_compatible = type_compatible;

      const double name_penalty = strict_match ? 0.0 : 0.10;
      for (size_t i = 0; i < pred_xyza.size(); ++i) {
        const auto & pred = pred_xyza[i];
        const double pos_diff = (armor.xyz_in_world - pred.head(3)).norm();
        const double yaw_diff = std::abs(tools::limit_rad(armor.ypr_in_world[0] - pred[3]));
        const double z_diff = std::abs(armor.xyz_in_world[2] - pred[2]);

        double reproj_err = std::numeric_limits<double>::infinity();
        if (armor.points.size() == 4) {
          const auto reproj =
            solver_.reproject_armor(pred.head(3), pred[3], ArmorType::small, target_.name);
          reproj_err = reprojection_error(armor.points, reproj);
        }

        const double score = 0.45 * (pos_diff / 0.24) + 0.28 * (yaw_diff / 1.2) +
                             1.10 * (z_diff / 0.07) + 0.95 * (reproj_err / 18.0) +
                             (relaxed_name ? 0.12 : 0.0) + (type_compatible ? 0.0 : 0.15) +
                             name_penalty;
        if (score < best.score) {
          best.id = static_cast<int>(i);
          best.pos_diff = pos_diff;
          best.yaw_diff = yaw_diff;
          best.z_diff = z_diff;
          best.reproj_err = reproj_err;
          best.score = score;
        }
      }

      if (best.id < 0) {
        continue;
      }

      int fail_mask = 0;
      if (best.type_compatible && best.pos_diff > pos_gate) fail_mask |= 1;
      if (best.yaw_diff > yaw_gate) fail_mask |= 2;
      if (best.reproj_err > reproj_gate || best.z_diff > z_gate) fail_mask |= 4;
      best.fail_mask = fail_mask;
      out.push_back(best);
    }

    std::sort(
      out.begin(), out.end(),
      [](const MatchCandidate & a, const MatchCandidate & b) { return a.score < b.score; });
    return out;
  };

  const auto strict_candidates = evaluate_candidates(false);
  const auto relaxed_candidates = evaluate_candidates(true);
  debug_info_.candidate_count = static_cast<int>(
    strict_candidates.empty() ? relaxed_candidates.size() : strict_candidates.size());

  if (strict_candidates.empty() && relaxed_candidates.empty()) {
    debug_info_.reject_code = 1;
    return false;
  }

  auto hard_accept = [](const MatchCandidate & c) {
    return c.id >= 0 && c.type_compatible && c.fail_mask == 0;
  };
  auto loose_accept = [](const MatchCandidate & c) {
    if (c.id < 0 || !std::isfinite(c.reproj_err)) return false;
    if (c.type_compatible) {
      return c.reproj_err < 18.0 && c.z_diff < 0.14;
    }
    return c.reproj_err < 12.0 && c.z_diff < 0.10 && c.pos_diff < 0.35;
  };

  const auto & primary_pool = strict_candidates.empty() ? relaxed_candidates : strict_candidates;
  const auto & primary_best = primary_pool.front();
  const double primary_margin = (primary_pool.size() >= 2) ? (primary_pool[1].score - primary_best.score) :
                                                         std::numeric_limits<double>::infinity();

  debug_info_.pre_gate_pos_diff = primary_best.pos_diff;
  debug_info_.pre_gate_yaw_diff = primary_best.yaw_diff;
  debug_info_.pre_gate_reproj_err = primary_best.reproj_err;
  debug_info_.pre_gate_score = primary_best.score;
  debug_info_.pre_gate_id = primary_best.id;
  debug_info_.pre_gate_fail_mask = primary_best.fail_mask;
  debug_info_.gate_distance = pos_gate;
  debug_info_.gate_yaw = yaw_gate;

  const double ambiguity_floor = std::max(outpost_ambiguity_reproj_floor_, 8.0);
  const bool ambiguous =
    std::isfinite(primary_margin) && primary_margin < outpost_min_match_margin_ &&
    primary_best.reproj_err > ambiguity_floor;

  const MatchCandidate * chosen = nullptr;
  bool force_switch = false;
  auto best_id_from_observation = [&](const Armor & obs) {
    int best_id = target_.last_id;
    double best_err = std::numeric_limits<double>::infinity();
    if (obs.points.size() != 4) {
      return std::make_pair(best_id, best_err);
    }
    const auto xyza_list = target_.armor_xyza_list();
    for (size_t i = 0; i < xyza_list.size(); ++i) {
      const auto reproj = solver_.reproject_armor(
        xyza_list[i].head(3), xyza_list[i][3], ArmorType::small, ArmorName::outpost);
      const double err = reprojection_error(obs.points, reproj);
      if (err < best_err) {
        best_err = err;
        best_id = static_cast<int>(i);
      }
    }
    return std::make_pair(best_id, best_err);
  };
  auto emergency_relock = [&](const MatchCandidate & c) {
    if (c.armor == nullptr || c.id < 0) {
      return false;
    }

    Armor relock_armor = *c.armor;
    relock_armor.name = ArmorName::outpost;
    relock_armor.type = ArmorType::small;
    solver_.solve(relock_armor);

    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(relock_armor, t, 0.275, 3, P0_dig);
    target_.armor_type = ArmorType::small;
    int init_id = 0;
    const auto init_guess = best_id_from_observation(relock_armor);
    if (init_guess.first >= 0) {
      init_id = init_guess.first;
    } else {
      double min_z_err = std::numeric_limits<double>::infinity();
      const auto relock_xyza = target_.armor_xyza_list();
      for (size_t i = 0; i < relock_xyza.size(); ++i) {
        const double z_err = std::abs(relock_armor.xyz_in_world[2] - relock_xyza[i][2]);
        if (z_err < min_z_err) {
          min_z_err = z_err;
          init_id = static_cast<int>(i);
        }
      }
    }
    target_.update(relock_armor, true, init_id);
    const auto post_match = best_id_from_observation(relock_armor);
    if (!std::isfinite(post_match.second) || post_match.second > 40.0) {
      return false;
    }

    debug_info_.matched = true;
    debug_info_.reject_code = 0;
    debug_info_.matched_pos_diff = std::isfinite(c.pos_diff) ? c.pos_diff : 0.0;
    debug_info_.matched_yaw_diff = std::isfinite(c.yaw_diff) ? c.yaw_diff : 0.0;
    debug_info_.matched_reproj_err = std::isfinite(post_match.second) ? post_match.second : c.reproj_err;
    debug_info_.matched_score = std::isfinite(c.score) ? c.score : 0.0;
    debug_info_.match_margin = std::numeric_limits<double>::infinity();
    debug_info_.matched_id = (post_match.first >= 0) ? post_match.first : target_.last_id;
    debug_info_.pre_gate_pos_diff = debug_info_.matched_pos_diff;
    debug_info_.pre_gate_yaw_diff = debug_info_.matched_yaw_diff;
    debug_info_.pre_gate_reproj_err = debug_info_.matched_reproj_err;
    debug_info_.pre_gate_score = debug_info_.matched_score;
    debug_info_.pre_gate_id = debug_info_.matched_id;
    debug_info_.pre_gate_fail_mask = 0;
    debug_info_.used_relaxed_jump = c.relaxed;
    debug_info_.used_emergency_relock = true;
    return true;
  };

  if (hard_accept(primary_best) && !ambiguous) {
    chosen = &primary_best;
  } else {
    if (!relaxed_candidates.empty()) {
      const auto & relaxed_best = relaxed_candidates.front();
      if (loose_accept(relaxed_best) &&
          (!ambiguous || (relaxed_best.score + 0.03 < primary_best.score))) {
        chosen = &relaxed_best;
        force_switch = true;
        debug_info_.used_relaxed_jump = !strict_candidates.empty();
      }
    }
    if (chosen == nullptr && !ambiguous && loose_accept(primary_best)) {
      chosen = &primary_best;
      force_switch = true;
      debug_info_.used_relaxed_jump = true;
    }
  }

  if (chosen == nullptr) {
    const bool huge_pre_relock =
      std::isfinite(primary_best.reproj_err) && primary_best.reproj_err > 18.0 &&
      primary_best.id >= 0 && primary_best.armor != nullptr;
    if (huge_pre_relock) {
      if (emergency_relock(primary_best)) {
        return true;
      }
    }

    const bool early_tracking_relock =
      (state_ == "tracking" || state_ == "temp_lost") && std::isfinite(primary_best.reproj_err) &&
      primary_best.reproj_err > 45.0 && !strict_candidates.empty() &&
      strict_candidates.front().id >= 0 && strict_candidates.front().armor != nullptr;
    if (early_tracking_relock) {
      if (emergency_relock(strict_candidates.front())) {
        return true;
      }
    }

    // Severe drift recovery: only relock from strict outpost candidate to avoid false relock.
    const bool severe_tracking_drift =
      std::isfinite(primary_best.reproj_err) && primary_best.reproj_err > 120.0 &&
      std::isfinite(primary_best.pos_diff) && primary_best.pos_diff > 0.45;
    const bool temp_lost_drift =
      (state_ == "temp_lost") && std::isfinite(primary_best.reproj_err) &&
      primary_best.reproj_err > 30.0;
    if (!strict_candidates.empty() &&
        ((state_ == "tracking" && severe_tracking_drift) || temp_lost_drift)) {
      if (emergency_relock(strict_candidates.front())) {
        return true;
      }
    }

    if ((state_ == "temp_lost" || state_ == "tracking") && !relaxed_candidates.empty()) {
      const auto & relaxed_best = relaxed_candidates.front();
      const bool relaxed_severe_drift_relock =
        strict_candidates.empty() && relaxed_best.id >= 0 && relaxed_best.armor != nullptr &&
        std::isfinite(relaxed_best.reproj_err) && relaxed_best.reproj_err > 55.0 &&
        std::isfinite(relaxed_best.z_diff) && relaxed_best.z_diff < 0.20 &&
        std::isfinite(relaxed_best.pos_diff) && relaxed_best.pos_diff < 0.50;
      if (relaxed_severe_drift_relock && emergency_relock(relaxed_best)) {
        return true;
      }
      const bool relaxed_relock_ok =
        relaxed_best.id >= 0 && relaxed_best.armor != nullptr &&
        std::isfinite(relaxed_best.reproj_err) && relaxed_best.reproj_err < 24.0 &&
        std::isfinite(relaxed_best.z_diff) && relaxed_best.z_diff < 0.060 &&
        std::isfinite(relaxed_best.pos_diff) && relaxed_best.pos_diff < 0.28;
      if (relaxed_relock_ok && emergency_relock(relaxed_best)) {
        return true;
      }
    }

    if (ambiguous) {
      debug_info_.ambiguous_reject = true;
      debug_info_.reject_code = 3;
    } else {
      debug_info_.reject_code = 2;
    }
    return false;
  }

  double chosen_margin = std::numeric_limits<double>::infinity();
  if (chosen->relaxed) {
    if (relaxed_candidates.size() >= 2) {
      chosen_margin = relaxed_candidates[1].score - relaxed_candidates[0].score;
    }
  } else if (strict_candidates.size() >= 2) {
    chosen_margin = strict_candidates[1].score - strict_candidates[0].score;
  }

  debug_info_.matched = true;
  debug_info_.reject_code = 0;
  debug_info_.matched_pos_diff = chosen->pos_diff;
  debug_info_.matched_yaw_diff = chosen->yaw_diff;
  debug_info_.matched_reproj_err = chosen->reproj_err;
  debug_info_.matched_score = chosen->score;
  debug_info_.match_margin = chosen_margin;
  debug_info_.matched_id = chosen->id;
  // Keep debug "pre-gate" metrics aligned with effective association when relaxed path wins.
  if (
    chosen->relaxed && std::isfinite(chosen->reproj_err) &&
    (!std::isfinite(debug_info_.pre_gate_reproj_err) || chosen->reproj_err < debug_info_.pre_gate_reproj_err)) {
    debug_info_.pre_gate_pos_diff = chosen->pos_diff;
    debug_info_.pre_gate_yaw_diff = chosen->yaw_diff;
    debug_info_.pre_gate_reproj_err = chosen->reproj_err;
    debug_info_.pre_gate_score = chosen->score;
    debug_info_.pre_gate_id = chosen->id;
    debug_info_.pre_gate_fail_mask = chosen->fail_mask;
  }

  target_.update(*chosen->armor, force_switch || chosen->relaxed, chosen->id);
  if (chosen->armor != nullptr && chosen->armor->points.size() == 4) {
    double post_min_err = std::numeric_limits<double>::infinity();
    int post_best_id = chosen->id;
    const auto post_xyza = target_.armor_xyza_list();
    for (size_t i = 0; i < post_xyza.size(); ++i) {
      const auto reproj =
        solver_.reproject_armor(post_xyza[i].head(3), post_xyza[i][3], ArmorType::small, ArmorName::outpost);
      const double err = reprojection_error(chosen->armor->points, reproj);
      if (err < post_min_err) {
        post_min_err = err;
        post_best_id = static_cast<int>(i);
      }
    }

    debug_info_.matched_reproj_err = post_min_err;
    debug_info_.matched_id = post_best_id;
    const bool post_diverged =
      !std::isfinite(post_min_err) || post_min_err > 24.0 ||
      (std::isfinite(chosen->reproj_err) && chosen->reproj_err > 10.0 &&
       post_min_err > chosen->reproj_err + 12.0);
    if (post_diverged && (state_ == "temp_lost" || post_min_err > 32.0)) {
      MatchCandidate relock_choice = *chosen;
      relock_choice.id = post_best_id >= 0 ? post_best_id : 0;
      if (emergency_relock(relock_choice)) {
        return true;
      }
    }
  }
  return true;
}

}  // namespace auto_aim
