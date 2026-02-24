#ifndef TRACK_PLANNING__CORRIDOR__PAIR_VALIDATOR_HPP_
#define TRACK_PLANNING__CORRIDOR__PAIR_VALIDATOR_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <vector>

namespace track_planning
{

class PairValidator
{
public:
  /// Validate left/right polyline pair
  /// Returns PairResult with validity, width statistics, and angle statistics
  PairResult validate(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    const PlanningParams & p);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__PAIR_VALIDATOR_HPP_
