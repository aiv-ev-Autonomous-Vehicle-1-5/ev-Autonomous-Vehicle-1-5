#ifndef TRACK_PLANNING__PLANNER__MODE_SELECTOR_HPP_
#define TRACK_PLANNING__PLANNER__MODE_SELECTOR_HPP_

#include "track_planning/common/types.hpp"

namespace track_planning
{
namespace mode_selector
{

/// Determine planning mode: DIRECT or ASTAR.
/// Triggers ASTAR if any of the following are true:
///   1. pair_valid == false
///   2. virtual_used == true
///   3. centerline_valid == false
///   4. centerline_collision == true (centerline hits validation costmap)
///   5. centerline_jump == true (inter-frame discontinuity)
/// If enable_astar == false, always returns DIRECT.
inline PathMode select(
  bool pair_valid,
  bool virtual_used,
  bool centerline_valid,
  bool centerline_collision,
  bool centerline_jump,
  bool enable_astar)
{
  if (!enable_astar) {
    return PathMode::DIRECT;
  }

  if (!pair_valid || virtual_used || !centerline_valid ||
      centerline_collision || centerline_jump)
  {
    return PathMode::ASTAR;
  }

  return PathMode::DIRECT;
}

}  // namespace mode_selector
}  // namespace track_planning

#endif  // TRACK_PLANNING__PLANNER__MODE_SELECTOR_HPP_
