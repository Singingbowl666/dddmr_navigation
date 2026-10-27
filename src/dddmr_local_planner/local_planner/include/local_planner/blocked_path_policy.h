#ifndef LOCAL_PLANNER_BLOCKED_PATH_POLICY_H_
#define LOCAL_PLANNER_BLOCKED_PATH_POLICY_H_

#include <cmath>
#include <cstddef>

namespace local_planner {
namespace detail {

enum class TrajectoryDecision { Execute, Wait, Replan, NoValidTrajectory };

// Apply this only after sensor/TF checks, trajectory generation and critic scoring.
// Explicit replanning requests retain priority over an alternative trajectory.
inline TrajectoryDecision selectTrajectoryDecision(
  bool path_blocked_wait, bool path_blocked_replanning,
  bool allow_valid_trajectory_when_path_blocked, double cost, std::size_t pose_count)
{
  if (path_blocked_replanning) {
    return TrajectoryDecision::Replan;
  }
  const bool valid = std::isfinite(cost) && cost >= 0.0 && pose_count > 0;
  if (path_blocked_wait && !(allow_valid_trajectory_when_path_blocked && valid)) {
    return TrajectoryDecision::Wait;
  }
  return valid ? TrajectoryDecision::Execute : TrajectoryDecision::NoValidTrajectory;
}

}  // namespace detail
}  // namespace local_planner

#endif
