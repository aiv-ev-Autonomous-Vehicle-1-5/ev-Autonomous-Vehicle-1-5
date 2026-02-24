#ifndef TRACK_PLANNING__COSTMAP__COSTMAP_VALIDATION_BUILDER_HPP_
#define TRACK_PLANNING__COSTMAP__COSTMAP_VALIDATION_BUILDER_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstdint>
#include <vector>

namespace track_planning
{

class CostmapValidationBuilder
{
public:
  struct Result
  {
    nav_msgs::msg::OccupancyGrid grid_msg;
    std::vector<int8_t> cost_array;  // internal cost buffer
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    bool valid = false;
  };

  /// Build validation costmap with boundary barrier + obstacle layers + inflation
  Result build(
    const std::vector<Point2D> & corridor_left,
    const std::vector<Point2D> & corridor_right,
    const std::vector<Point2D> & cones,
    const std::vector<Point2D> & obstacles,
    const PlanningParams & p);

  /// Check if centerline collides with the costmap
  /// @return true if collision detected (DIRECT mode should fail)
  static bool check_centerline_collision(
    const Result & costmap,
    const std::vector<Point2D> & centerline,
    double ds_check,
    int cost_th);

private:
  // World coordinate → grid cell index
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y, double resolution,
    int width, int height,
    int & col, int & row);

  // Rasterize a polyline as connected line segments
  static void rasterize_polyline(
    std::vector<int8_t> & grid, int width, int height,
    const std::vector<Point2D> & pts,
    double resolution, double origin_x, double origin_y,
    int8_t value);

  // Rasterize individual points (single cell each)
  static void rasterize_points(
    std::vector<int8_t> & grid, int width, int height,
    const std::vector<Point2D> & pts,
    double resolution, double origin_x, double origin_y,
    int8_t value);

  // Bresenham line drawing
  static void draw_line(
    std::vector<int8_t> & grid, int width, int height,
    int x0, int y0, int x1, int y1,
    int8_t value);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__COSTMAP_VALIDATION_BUILDER_HPP_
