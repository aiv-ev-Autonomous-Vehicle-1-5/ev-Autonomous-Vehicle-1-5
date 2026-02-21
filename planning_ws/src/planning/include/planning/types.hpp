#pragma once
#include <vector>

namespace planning
{

struct State
{
    double x{0.0};
    double y{0.0};
    double theta{0.0}; // rad
};

using Grid = std::vector<std::vector<int>>; // grid[x][y], 0 free, 1 occupied

} // namespace planning