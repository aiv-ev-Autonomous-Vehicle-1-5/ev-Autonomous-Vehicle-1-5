#include "planning/collision_checker.hpp"
#include <cmath>

namespace planning{

CollisionChecker::CollisionChecker(double resolution, int width, int height)
    : resolution_(resolution), width_(width), height_(height)
{}

bool CollisionChecker::isCollision(double x, double y, const Grid& grid)
{
    int gx = static_cast<int>(x / resolution_);
    int gy = static_cast<int>(y / resolution_);

    if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_)
        return true;

    return grid[gx][gy] == 1;
}

}