#include "planning/hybrid_astar.hpp"

#include <cmath>
#include <queue>
#include <limits>
#include <tuple>
#include <algorithm>

namespace planning
{

struct SearchNode
{
    State s;
    double g{0.0};
    double h{0.0};
    int parent{-1};
};

struct PQItem
{
    double f;
    int idx;
    bool operator<(const PQItem& other) const { return f > other.f; } // min-heap
};

HybridAStar::HybridAStar(double wheelbase,
                         double delta_max,
                         double step_size,
                         int theta_bins,
                         double grid_resolution)
    : wheelbase_(wheelbase),
      delta_max_(delta_max),
      step_(step_size),
      theta_bins_(theta_bins),
      res_(grid_resolution)
{}

double HybridAStar::normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double HybridAStar::angleDiff(double a, double b)
{
    return normalizeAngle(a - b);
}

int HybridAStar::thetaToBin(double theta) const
{
    double t = normalizeAngle(theta);
    // map [-pi, pi) -> [0, theta_bins)
    double ratio = (t + M_PI) / (2.0 * M_PI);
    int bin = static_cast<int>(std::floor(ratio * theta_bins_));
    if (bin < 0) bin = 0;
    if (bin >= theta_bins_) bin = theta_bins_ - 1;
    return bin;
}

double HybridAStar::heuristic(const State& a, const State& b) const
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool HybridAStar::isMotionCollisionFree(const State& from,
                                        const State& to,
                                        const Grid& grid,
                                        CollisionChecker& checker) const
{
    // check along segment in small steps (simple + robust)
    const int N = 10;
    for (int i = 0; i <= N; ++i)
    {
        double t = static_cast<double>(i) / N;
        double x = from.x + t * (to.x - from.x);
        double y = from.y + t * (to.y - from.y);
        if (checker.isCollision(x, y, grid)) return false;
    }
    return true;
}

std::vector<State> HybridAStar::plan(const State& start,
                                     const State& goal,
                                     const Grid& grid)
{
    std::vector<State> empty;

    if (grid.empty() || grid[0].empty()) return empty;

    const int W = static_cast<int>(grid.size());
    const int H = static_cast<int>(grid[0].size());

    CollisionChecker checker(res_, W, H);

    // 3D best g-cost map
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> best_g(W * H * theta_bins_, INF);

    auto toIndex = [&](int ix, int iy, int it) {
        return (it * W * H) + (ix * H) + iy;
    };

    auto worldToGrid = [&](double x, double y, int& ix, int& iy) -> bool {
        ix = static_cast<int>(std::floor(x / res_));
        iy = static_cast<int>(std::floor(y / res_));
        if (ix < 0 || iy < 0 || ix >= W || iy >= H) return false;
        return true;
    };

    // start validity
    int sx, sy;
    if (!worldToGrid(start.x, start.y, sx, sy)) return empty;
    if (grid[sx][sy] == 1) return empty;

    // steering samples (5)
    const std::vector<double> deltas = {
        -delta_max_,
        -0.5 * delta_max_,
        0.0,
        0.5 * delta_max_,
        delta_max_
    };

    // open set
    std::priority_queue<PQItem> open;
    std::vector<SearchNode> nodes;
    nodes.reserve(50000);

    SearchNode s0;
    s0.s = start;
    s0.s.theta = normalizeAngle(s0.s.theta);
    s0.g = 0.0;
    s0.h = heuristic(s0.s, goal);
    s0.parent = -1;

    int st = thetaToBin(s0.s.theta);
    int si = toIndex(sx, sy, st);
    best_g[si] = 0.0;

    nodes.push_back(s0);
    open.push({s0.g + s0.h, 0});

    // goal condition
    const double goal_dist_tol = 0.30;         // meters
    const double goal_yaw_tol  = 20.0 * M_PI/180.0; // rad

    int goal_node_idx = -1;

    while (!open.empty())
    {
        auto curItem = open.top();
        open.pop();

        const int curIdx = curItem.idx;
        const SearchNode& cur = nodes[curIdx];

        // check goal
        if (heuristic(cur.s, goal) < goal_dist_tol &&
            std::fabs(angleDiff(cur.s.theta, goal.theta)) < goal_yaw_tol)
        {
            goal_node_idx = curIdx;
            break;
        }

        // expand neighbors
        for (double delta : deltas)
        {
            // bicycle model forward step (arc length ~ step_)
            State nxt = cur.s;
            nxt.x += step_ * std::cos(cur.s.theta);
            nxt.y += step_ * std::sin(cur.s.theta);
            nxt.theta = normalizeAngle(cur.s.theta + (step_ / wheelbase_) * std::tan(delta));

            int nx, ny;
            if (!worldToGrid(nxt.x, nxt.y, nx, ny)) continue;
            if (grid[nx][ny] == 1) continue;

            // collision along motion
            if (!isMotionCollisionFree(cur.s, nxt, grid, checker)) continue;

            // costs
            double turn_penalty = std::fabs(delta) / (delta_max_ + 1e-6); // [0..1]
            double new_g = cur.g + step_ * (1.0 + 0.1 * turn_penalty);

            int nt = thetaToBin(nxt.theta);
            int bi = toIndex(nx, ny, nt);

            if (new_g + 1e-9 >= best_g[bi]) continue;
            best_g[bi] = new_g;

            SearchNode nn;
            nn.s = nxt;
            nn.g = new_g;
            nn.h = heuristic(nxt, goal);
            nn.parent = curIdx;

            int newIdx = static_cast<int>(nodes.size());
            nodes.push_back(nn);
            open.push({nn.g + nn.h, newIdx});
        }
    }

    if (goal_node_idx < 0) return empty;

    // reconstruct
    std::vector<State> path;
    for (int idx = goal_node_idx; idx >= 0; idx = nodes[idx].parent)
        path.push_back(nodes[idx].s);

    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace planning