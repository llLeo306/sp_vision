#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  struct DebugInfo
  {
    bool matched = false;
    bool used_relaxed_jump = false;
    bool used_emergency_relock = false;
    bool ambiguous_reject = false;
    int candidate_count = 0;
    int reject_code = 0;  // 0=none,1=no_candidate,2=gated_out,3=ambiguous
    double matched_pos_diff = 0.0;
    double matched_yaw_diff = 0.0;
    double matched_reproj_err = 0.0;
    double matched_score = 0.0;
    double match_margin = 0.0;
    int matched_id = -1;
    double gate_distance = 0.0;
    double gate_yaw = 0.0;
    double pre_gate_pos_diff = 0.0;
    double pre_gate_yaw_diff = 0.0;
    double pre_gate_reproj_err = 0.0;
    double pre_gate_score = 0.0;
    int pre_gate_id = -1;
    int pre_gate_fail_mask = 0;  // bit0=pos bit1=yaw bit2=reproj bit3=invalid_id
  };

  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;
  const DebugInfo & debug_info() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  double max_match_distance_;
  double max_match_yaw_diff_;
  double outpost_extra_match_yaw_diff_;
  double outpost_min_match_margin_;
  double outpost_ambiguity_reproj_floor_;
  int outpost_top_relock_streak_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  DebugInfo debug_info_;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
