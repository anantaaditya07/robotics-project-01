// Hand-computed tests for the ROS-free semantic cost model (architecture 7.3).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "semnav_costmap/cost_model.hpp"

using semnav_costmap::CellWindow;
using semnav_costmap::ClassCostParams;
using semnav_costmap::combine;
using semnav_costmap::cost_at;
using semnav_costmap::footprint_window;
using semnav_costmap::GridView;
using semnav_costmap::paint_disc;

namespace {

// person: r 0.35 m, inflation 1.0 m (7.3), k 3.0 /m (layer default).
const ClassCostParams kPerson{0.35, 1.0, 3.0};
constexpr double kEps = 1e-6;

// 10 x 10 grid, 0.1 m cells, origin (0, 0): cell centres at 0.05 + 0.1 * i.
struct Grid {
  std::vector<std::uint8_t> cells;
  GridView view;
  explicit Grid(std::uint8_t fill, int w = 10, int h = 10, double res = 0.1)
      : cells(static_cast<std::size_t>(w * h), fill) {
    view = GridView{cells.data(), w, h, res, 0.0, 0.0};
  }
  int count(std::uint8_t v) const {
    int n = 0;
    for (auto c : cells) {
      n += c == v ? 1 : 0;
    }
    return n;
  }
  std::uint8_t at(int i, int j) const {
    return cells[static_cast<std::size_t>(j * view.width + i)];
  }
};

}  // namespace

TEST(CostAt, CoreIsLethal) {
  EXPECT_EQ(cost_at(0.0, kPerson), 254);
  EXPECT_EQ(cost_at(0.2, kPerson), 254);
  EXPECT_EQ(cost_at(0.35, kPerson), 254);  // d == r is still core
}

TEST(CostAt, JustPastRadiusIs252) {
  // 252 * exp(-3 * 1e-6) = 251.99924 -> rounds to 252, never 253/254.
  EXPECT_EQ(cost_at(0.35 + kEps, kPerson), 252);
}

TEST(CostAt, OuterEdgeOfInflation) {
  // d = r + inflation = 1.35: 252 * exp(-3 * 1.0) = 12.546 -> 13.
  EXPECT_EQ(cost_at(1.35, kPerson), 13);
  EXPECT_EQ(cost_at(1.35, kPerson), static_cast<std::uint8_t>(std::round(252.0 * std::exp(-3.0))));
  // Mid band, d - r = 0.5: 252 * exp(-1.5) = 56.229 -> 56.
  EXPECT_EQ(cost_at(0.85, kPerson), 56);
}

TEST(CostAt, BeyondInflationIsZero) {
  EXPECT_EQ(cost_at(1.35 + kEps, kPerson), 0);
  EXPECT_EQ(cost_at(5.0, kPerson), 0);
}

TEST(CostAt, MonotonicNonIncreasingAndNever253InDecay) {
  std::uint8_t prev = cost_at(0.0, kPerson);
  for (double d = 0.0; d <= 2.0; d += 0.001) {
    const std::uint8_t c = cost_at(d, kPerson);
    EXPECT_LE(c, prev) << "d=" << d;
    if (d > kPerson.radius) {
      EXPECT_LE(c, 252) << "d=" << d;
    }
    prev = c;
  }
}

TEST(CostAt, ZeroDecayKeeps252AcrossBand) {
  const ClassCostParams flat{0.25, 0.3, 0.0};  // chair r/inflation, k = 0
  EXPECT_EQ(cost_at(0.26, flat), 252);
  EXPECT_EQ(cost_at(0.55, flat), 252);
  EXPECT_EQ(cost_at(0.56, flat), 0);
}

TEST(Combine, MaxNeverLowersExistingCost) {
  EXPECT_EQ(combine(254, 100), 254);
  EXPECT_EQ(combine(253, 252), 253);
  EXPECT_EQ(combine(50, 100), 100);
  EXPECT_EQ(combine(0, 254), 254);
  EXPECT_EQ(combine(120, 0), 120);
}

TEST(Combine, UnknownIsLowest) {
  // Documented choice: NO_INFORMATION (255) is overwritten by any non-zero semantic cost,
  // and a zero semantic cost leaves it unknown.
  EXPECT_EQ(combine(255, 100), 100);
  EXPECT_EQ(combine(255, 254), 254);
  EXPECT_EQ(combine(255, 0), 255);
}

TEST(FootprintWindow, CentredDisc) {
  // (0.5, 0.5), reach 0.35: floor(1.5) = 1 .. floor(8.5) = 8 -> [1, 9).
  const CellWindow w = footprint_window(0.5, 0.5, 0.35, 10, 10, 0.1, 0.0, 0.0);
  EXPECT_EQ(w.min_i, 1);
  EXPECT_EQ(w.min_j, 1);
  EXPECT_EQ(w.max_i, 9);
  EXPECT_EQ(w.max_j, 9);
}

TEST(FootprintWindow, ClippedAtBorderAndOutside) {
  // (0, 0), reach 0.35: floor(-3.5) = -4 -> 0, floor(3.5) + 1 = 4.
  const CellWindow w = footprint_window(0.0, 0.0, 0.35, 10, 10, 0.1, 0.0, 0.0);
  EXPECT_EQ(w.min_i, 0);
  EXPECT_EQ(w.min_j, 0);
  EXPECT_EQ(w.max_i, 4);
  EXPECT_EQ(w.max_j, 4);
  EXPECT_TRUE(footprint_window(-5.0, -5.0, 0.35, 10, 10, 0.1, 0.0, 0.0).empty());
  EXPECT_TRUE(footprint_window(2.0, 0.5, 0.35, 10, 10, 0.1, 0.0, 0.0).empty());  // past x = 1.0
}

// Person core r = 0.35 centred on the grid corner point (0.5, 0.5). Cell-centre offsets are
// +/-0.05, 0.15, 0.25, 0.35. Per quadrant, cells with dx^2 + dy^2 <= 0.1225:
//   |dx| = 0.05: |dy| <= 0.346 -> 0.05, 0.15, 0.25 (3)
//   |dx| = 0.15: |dy| <= 0.316 -> 3
//   |dx| = 0.25: |dy| <= 0.245 -> 0.05, 0.15 (2)
//   |dx| = 0.35: |dy| <= 0     -> none
// = 8 per quadrant, 32 lethal cells.
TEST(PaintDisc, PersonCoreLethalCount) {
  Grid g(0);
  paint_disc(g.view, 0.5, 0.5, ClassCostParams{0.35, 0.0, 3.0});
  EXPECT_EQ(g.count(254), 32);
  EXPECT_EQ(g.count(0), 100 - 32);
}

TEST(PaintDisc, DecayNeverAddsLethalCells) {
  Grid g(0);
  paint_disc(g.view, 0.5, 0.5, kPerson);
  EXPECT_EQ(g.count(254), 32);
  EXPECT_EQ(g.count(253), 0);
  EXPECT_EQ(g.count(0), 0);  // whole 1 m x 1 m grid is within r + 1.0 of the centre
  // Corner cell (0, 0): centre (0.05, 0.05), d = 0.6364, d - r = 0.2864:
  // 252 * exp(-0.8592) = 106.71 -> 107.
  EXPECT_EQ(g.at(0, 0), 107);
}

TEST(PaintDisc, NeverLowersHigherExistingCost) {
  Grid g(253);
  paint_disc(g.view, 0.5, 0.5, kPerson);
  EXPECT_EQ(g.count(254), 32);
  EXPECT_EQ(g.count(253), 100 - 32);
}

TEST(PaintDisc, ClippedAtGridBorder) {
  // Centre on the grid corner: only one quadrant (8 cells) of the core lies inside.
  Grid g(0);
  paint_disc(g.view, 0.0, 0.0, ClassCostParams{0.35, 0.0, 3.0});
  EXPECT_EQ(g.count(254), 8);
  // Entirely outside: untouched.
  Grid h(0);
  paint_disc(h.view, -5.0, -5.0, kPerson);
  EXPECT_EQ(h.count(0), 100);
}

TEST(PaintDisc, ClippedToUpdateWindow) {
  // Only columns i in [0, 5) may be written: half of the 32 core cells.
  Grid g(0);
  paint_disc(g.view, 0.5, 0.5, ClassCostParams{0.35, 0.0, 3.0}, CellWindow{0, 0, 5, 10});
  EXPECT_EQ(g.count(254), 16);
  for (int j = 0; j < 10; ++j) {
    for (int i = 5; i < 10; ++i) {
      EXPECT_EQ(g.at(i, j), 0);
    }
  }
}

TEST(PaintDisc, UnknownCellsTakeSemanticCost) {
  // chair: r 0.25, inflation 0.3 -> reach 0.55. Unknown inside the reach is overwritten,
  // unknown beyond stays 255.
  Grid g(255);
  const ClassCostParams chair{0.25, 0.3, 3.0};
  paint_disc(g.view, 0.5, 0.5, chair);
  // Cell (4, 4): centre (0.45, 0.45), d = 0.0707 -> lethal.
  EXPECT_EQ(g.at(4, 4), 254);
  // Cell (6, 4): centre (0.65, 0.45), d = 0.1581 -> lethal.
  EXPECT_EQ(g.at(6, 4), 254);
  // Cell (7, 4): centre (0.75, 0.45), d = 0.2550, d - r = 0.0050: 252 * exp(-0.0150) = 248.25.
  EXPECT_EQ(g.at(7, 4), 248);
  // Cell (0, 0): centre (0.05, 0.05), d = 0.6364 > 0.55 -> unknown kept.
  EXPECT_EQ(g.at(0, 0), 255);
}
