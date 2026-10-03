// Per-class semantic cost model and disc painting (architecture 7.3).
// Plain STL, no ROS or Nav2 dependency, so it is unit-testable on its own.
//
// Accepted deviation from 7.3 (D-26): 7.3 paints the core LETHAL (254) and decays from 252.
// Here semantic cost is NEVER lethal or inscribed: the core gets max_cost (1..252) and the ring
// decays from max_cost. LiDAR (the obstacle layer) is the only lethal source; a semantic
// obstacle only biases the planner, so a ghost track can no longer block a route.
#ifndef SEMNAV_COSTMAP__COST_MODEL_HPP_
#define SEMNAV_COSTMAP__COST_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace semnav_costmap {

// Cost values, identical to nav2_costmap_2d/cost_values.hpp.
inline constexpr std::uint8_t kFreeSpace = 0;
inline constexpr std::uint8_t kMaxNonObstacle = 252;
inline constexpr std::uint8_t kInscribedInflated = 253;
inline constexpr std::uint8_t kLethal = 254;
inline constexpr std::uint8_t kNoInformation = 255;

// Default semantic cap, same as the layer's max_semantic_cost default. Non-lethal by design.
inline constexpr std::uint8_t kDefaultMaxSemanticCost = 200;

struct ClassCostParams {
  double radius = 0.0;     // m, core radius painted max_cost (D-26, not LETHAL)
  double inflation = 0.0;  // m, decay band width beyond the core
  double k = 0.0;          // 1/m, decay rate in max_cost*exp(-k*(d-r))
  // Core cost and decay start. Valid range 1..252 (checked by the layer); cost_at() also caps it
  // at 252 so a semantic cost can never be INSCRIBED (253) or LETHAL (254).
  std::uint8_t max_cost = kDefaultMaxSemanticCost;
};

// Valid range for the layer's max_semantic_cost parameter: 1..252 (below INSCRIBED 253).
inline bool valid_max_semantic_cost(int v) {
  return v >= 1 && v <= static_cast<int>(kMaxNonObstacle);
}

// Cost at distance d (m) from the obstacle centre, with m = min(max_cost, 252):
//   d <= r                 -> m
//   r < d <= r + inflation -> round(m*exp(-k*(d-r))), clamped to [0, m] (continuous at r)
//   beyond                 -> 0 (nothing to paint)
// Never 253/254: LiDAR is the only lethal source (D-26).
inline std::uint8_t cost_at(double d, const ClassCostParams& p) {
  const std::uint8_t cap = std::min(p.max_cost, kMaxNonObstacle);
  if (d <= p.radius) {
    return cap;
  }
  if (d > p.radius + p.inflation) {
    return kFreeSpace;
  }
  const double m = static_cast<double>(cap);
  const double c = m * std::exp(-p.k * (d - p.radius));
  return static_cast<std::uint8_t>(std::clamp(std::round(c), 0.0, m));
}

// Combine an existing master-grid cost with a semantic cost. std::max, so an existing cost is
// never lowered. NO_INFORMATION (255) is treated as the LOWEST value: an unknown cell under a
// semantic obstacle takes our cost (same convention as Nav2's CostmapLayer::updateWithMax).
// A semantic cost of 0 never touches the cell (unknown stays unknown, free stays free).
inline std::uint8_t combine(std::uint8_t existing, std::uint8_t semantic) {
  if (semantic == kFreeSpace) {
    return existing;
  }
  if (existing == kNoInformation) {
    return semantic;
  }
  return std::max(existing, semantic);
}

// Half-open cell window [min_i, max_i) x [min_j, max_j), same convention as Nav2 updateCosts.
struct CellWindow {
  int min_i = 0;
  int min_j = 0;
  int max_i = 0;
  int max_j = 0;
  bool empty() const { return min_i >= max_i || min_j >= max_j; }
};

inline CellWindow intersect(const CellWindow& a, const CellWindow& b) {
  return CellWindow{std::max(a.min_i, b.min_i), std::max(a.min_j, b.min_j),
                    std::min(a.max_i, b.max_i), std::min(a.max_j, b.max_j)};
}

// Row-major grid (index = j * width + i), cell (i, j) covers
// [origin_x + i*res, origin_x + (i+1)*res) x [origin_y + j*res, origin_y + (j+1)*res).
// Same layout as nav2_costmap_2d::Costmap2D::getCharMap().
struct GridView {
  std::uint8_t* data = nullptr;
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  CellWindow full() const { return CellWindow{0, 0, width, height}; }
};

// Cells that a disc of radius `reach` centred at world (wx, wy) can touch, clipped to the grid.
// Empty if the disc is entirely outside the grid.
inline CellWindow footprint_window(double wx, double wy, double reach, int width, int height,
                                   double resolution, double origin_x, double origin_y) {
  const auto cell = [resolution](double w, double o) {
    return static_cast<int>(std::floor((w - o) / resolution));
  };
  CellWindow win{cell(wx - reach, origin_x), cell(wy - reach, origin_y),
                 cell(wx + reach, origin_x) + 1, cell(wy + reach, origin_y) + 1};
  return intersect(win, CellWindow{0, 0, width, height});
}

// Paint one obstacle's cost disc into the grid, inside `clip` only, combining with combine().
// Distance is measured from the obstacle centre to each cell centre.
inline void paint_disc(const GridView& g, double wx, double wy, const ClassCostParams& p,
                       const CellWindow& clip) {
  if (g.data == nullptr || !(g.resolution > 0.0)) {
    return;
  }
  const double reach = p.radius + p.inflation;
  const CellWindow win = intersect(intersect(footprint_window(wx, wy, reach, g.width, g.height,
                                                              g.resolution, g.origin_x, g.origin_y),
                                             clip),
                                   g.full());
  if (win.empty()) {
    return;
  }
  for (int j = win.min_j; j < win.max_j; ++j) {
    const double cy = g.origin_y + (static_cast<double>(j) + 0.5) * g.resolution;
    for (int i = win.min_i; i < win.max_i; ++i) {
      const double cx = g.origin_x + (static_cast<double>(i) + 0.5) * g.resolution;
      const std::uint8_t c = cost_at(std::hypot(cx - wx, cy - wy), p);
      std::uint8_t& cell = g.data[static_cast<std::size_t>(j) * static_cast<std::size_t>(g.width) +
                                  static_cast<std::size_t>(i)];
      cell = combine(cell, c);
    }
  }
}

inline void paint_disc(const GridView& g, double wx, double wy, const ClassCostParams& p) {
  paint_disc(g, wx, wy, p, g.full());
}

}  // namespace semnav_costmap

#endif  // SEMNAV_COSTMAP__COST_MODEL_HPP_
