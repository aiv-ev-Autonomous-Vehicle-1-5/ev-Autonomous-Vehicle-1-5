#ifndef TRACK_PLANNING__COSTMAP__INFLATION_HPP_
#define TRACK_PLANNING__COSTMAP__INFLATION_HPP_

#include <cstdint>
#include <vector>

namespace track_planning
{
namespace inflation
{

/// Inflate occupied cells in a 2D grid using a circular kernel
/// @param grid           row-major grid (size = width * height)
/// @param width          grid width (columns)
/// @param height         grid height (rows)
/// @param resolution     meters per cell
/// @param radius         inflation radius in meters
/// @param occupied_th    cells with value >= this are considered occupied source
/// @param inflated_value value to write into inflated cells (cells already >= occupied_th are kept)
void inflate_grid(
  std::vector<int8_t> & grid,
  int width,
  int height,
  double resolution,
  double radius,
  int8_t occupied_th,
  int8_t inflated_value);

}  // namespace inflation
}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__INFLATION_HPP_
