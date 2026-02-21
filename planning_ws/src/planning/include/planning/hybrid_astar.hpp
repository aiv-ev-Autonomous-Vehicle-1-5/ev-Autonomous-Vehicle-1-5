#pragma once
#include "planning/types.hpp"
#include "planning/collision_checker.hpp"

#include <vector>

namespace planning
{

class HybridAStar
{
public:
    HybridAStar(double wheelbase,
                double delta_max,
                double step_size,
                int theta_bins,
                double grid_resolution);

    std::vector<State> plan(const State& start,
                            const State& goal,
                            const Grid& grid);

private:
    // params
    double wheelbase_;
    double delta_max_;
    double step_;
    int theta_bins_;
    double res_;

    // helpers
    static double normalizeAngle(double a);
    static double angleDiff(double a, double b);

    double heuristic(const State& a, const State& b) const;

    bool isMotionCollisionFree(const State& from,
                               const State& to,
                               const Grid& grid,
                               CollisionChecker& checker) const;

    int thetaToBin(double theta) const;
};

} // namespace planning