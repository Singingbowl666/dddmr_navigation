#include <local_planner/blocked_path_policy.h>

#include <iostream>
#include <limits>

int main()
{
  using local_planner::detail::selectTrajectoryDecision;
  using D = local_planner::detail::TrajectoryDecision;
  struct Case {
    const char* name;
    bool wait, replan, allow;
    double cost;
    std::size_t poses;
    D expected;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const Case cases[] = {
    {"clear path executes normally", false, false, false, 1.0, 10, D::Execute},
    {"blocked reference permits approved alternative", true, false, true, 1.0, 10, D::Execute},
    {"zero cost alternative is valid", true, false, true, 0.0, 10, D::Execute},
    {"opt out preserves waiting", true, false, false, 1.0, 10, D::Wait},
    {"collision rejected candidate cannot bypass wait", true, false, true, -1.0, 10, D::Wait},
    {"no selected candidate cannot bypass wait", true, false, true, -1.0, 0, D::Wait},
    {"empty candidate cannot bypass wait", true, false, true, 1.0, 0, D::Wait},
    {"NaN cost cannot bypass wait", true, false, true, nan, 10, D::Wait},
    {"infinite cost cannot bypass wait", true, false, true, inf, 10, D::Wait},
    {"unblocked rejected candidate fails", false, false, true, -1.0, 10, D::NoValidTrajectory},
    {"unblocked empty candidate fails", false, false, true, 0.0, 0, D::NoValidTrajectory},
    {"unblocked NaN candidate fails", false, false, true, nan, 10, D::NoValidTrajectory},
    {"explicit replan overrides approved candidate", false, true, true, 1.0, 10, D::Replan},
    {"explicit replan overrides simultaneous wait", true, true, true, 1.0, 10, D::Replan},
    {"explicit replan retained without candidate", true, true, true, -1.0, 0, D::Replan},
  };
  unsigned passed = 0;
  for (const auto& c : cases) {
    if (selectTrajectoryDecision(c.wait, c.replan, c.allow, c.cost, c.poses) != c.expected) {
      std::cerr << "FAIL: " << c.name << '\n';
      return 1;
    }
    ++passed;
  }
  std::cout << "Passed " << passed << " blocked-path policy regression cases\n";
  return 0;
}
