#pragma once
#include "planning/types.hpp"

namespace planning{

class CollisionChecker{
    public:
        CollisionChecker(double resolution, int width, int height);

    bool isCollision(double x, double y, const Grid& grid);

    private:
        double resolution_;
        int width_;
        int height_;
    };

}