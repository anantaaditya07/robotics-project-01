// Hand-computed tests for the ROS-free semantic cost model (architecture 7.3, with deviation D-26:
// the core is max_cost (default 200), not LETHAL, and the ring decays from max_cost; LiDAR is the
// only lethal source).
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
using semnav_costmap::valid_max_semantic_cost;

namespace {

// person: r 0.35 m, inflation 1.0 m (7.3), k 3.0 /m, max_cost 200 (layer defaults).
const ClassCostParams kPerson{0.35, 1.0, 3.0, 200};
// Core only (no decay band), same r / k / max_cost.
const ClassCostParams kPersonCore{0.35, 0.0, 3.0, 200};
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

TEST(CostAt, CoreIsMaxCostNotLethal) {
  // D-26: was 254 (LETHAL) under 7.3; now max_cost = 200.
  EXPECT_EQ(cost_at(0.0, kPerson), 200);
  EXPECT_EQ(cost_at(0.2, kPerson), 200);
  EXPECT_EQ(cost_at(0.35, kPerson), 200);  // d == r is still core
  // Core follows max_cost exactly.
  for (int m : {1, 50, 128, 200, 252}) {
    const ClassCostParams p{0.35, 1.0, 3.0, static_cast<std::uint8_t>(m)};
    EXPECT_EQ(cost_at(0.0, p), m);
    EXPECT_EQ(cost_at(0.35, p), m);
  }
}

TEST(CostAt, JustPastRadiusIsMaxCost) {
  // Was 252 * exp(-3e-6) = 251.99924 -> 252. Now 200 * exp(-3e-6) = 199.99940 -> 200.
  EXPECT_EQ(cost_at(0.35 + kEps, kPerson), 200);
}

TEST(CostAt, RingContinuousAtRadius) {
  // The ring starts at max_cost, so the step across d = r is at most rounding (1 unit).
  for (int m : {1, 100, 200, 252}) {
    const ClassCostParams p{0.35, 1.0, 3.0, static_cast<std::uint8_t>(m)};
    EXPECT_EQ(cost_at(0.35, p), m);
    EXPECT_EQ(cost_at(0.35 + kEps, p), m) << "m=" << m;
    // d - r = 1e-3: m * exp(-3e-3) = m * 0.997004 -> within 1 of m for m <= 252 (252 -> 251.245).
    EXPECT_LE(m - static_cast<int>(cost_at(0.35 + 1e-3, p)), 1) << "m=" << m;
  }
}

TEST(CostAt, OuterEdgeOfInflation) {
  // d = r + inflation = 1.35: was 252 * exp(-3) = 12.546 -> 13; now 200 * exp(-3) = 9.957 -> 10.
  EXPECT_EQ(cost_at(1.35, kPerson), 10);
  EXPECT_EQ(cost_at(1.35, kPerson), static_cast<std::uint8_t>(std::round(200.0 * std::exp(-3.0))));
  // Mid band, d - r = 0.5: was 252 * exp(-1.5) = 56.229 -> 56; now 200 * exp(-1.5) = 44.626 -> 45.
  EXPECT_EQ(cost_at(0.85, kPerson), 45);
}

TEST(CostAt, BeyondInflationIsZero) {
  EXPECT_EQ(cost_at(1.35 + kEps, kPerson), 0);
  EXPECT_EQ(cost_at(5.0, kPerson), 0);
}

TEST(CostAt, MonotonicNonIncreasingAndCappedByMaxCost) {
  std::uint8_t prev = cost_at(0.0, kPerson);
  for (double d = 0.0; d <= 2.0; d += 0.001) {
    const std::uint8_t c = cost_at(d, kPerson);
    EXPECT_LE(c, prev) << "d=" << d;
    EXPECT_LE(c, 200) << "d=" << d;  // max_cost everywhere, core included
    prev = c;
  }
}

TEST(CostAt, NeverInscribedOrLethalAtTopOfRange) {
  // max_cost 252 (top of the valid range) and k = 0 (no decay): still never 253/254 at any d.
  for (double k : {0.0, 3.0}) {
    const ClassCostParams p{0.35, 1.0, k, 252};
    for (double d = 0.0; d <= 2.0; d += 0.001) {
      EXPECT_LT(cost_at(d, p), 253) << "k=" << k << " d=" << d;
    }
    EXPECT_EQ(cost_at(0.0, p), 252);
  }
  // Defensive cap: an out-of-range max_cost that bypassed the layer check is clamped to 252.
  for (int m : {253, 254, 255}) {
    const ClassCostParams p{0.35, 1.0, 0.0, static_cast<std::uint8_t>(m)};
    EXPECT_EQ(cost_at(0.0, p), 252) << "m=" << m;
    EXPECT_EQ(cost_at(0.5, p), 252) << "m=" << m;
  }
}

TEST(CostAt, ZeroDecayKeepsMaxCostAcrossBand) {
  const ClassCostParams flat{0.25, 0.3, 0.0, 252};  // chair r/inflation, k = 0, max_cost 252
  EXPECT_EQ(cost_at(0.26, flat), 252);
  EXPECT_EQ(cost_at(0.55, flat), 252);
  EXPECT_EQ(cost_at(0.56, flat), 0);
  const ClassCostParams flat200{0.25, 0.3, 0.0, 200};
  EXPECT_EQ(cost_at(0.26, flat200), 200);
  EXPECT_EQ(cost_at(0.55, flat200), 200);
}

TEST(CostAt, DefaultMaxCostIs200) {
  const ClassCostParams p{0.35, 1.0, 3.0};  // max_cost omitted -> default
  EXPECT_EQ(p.max_cost, 200);
  EXPECT_EQ(cost_at(0.0, p), 200);
}

TEST(MaxSemanticCost, ValidRangeIs1To252) {
  EXPECT_FALSE(valid_max_semantic_cost(-1));
  EXPECT_FALSE(valid_max_semantic_cost(0));
  EXPECT_TRUE(valid_max_semantic_cost(1));
  EXPECT_TRUE(valid_max_semantic_cost(200));
  EXPECT_TRUE(valid_max_semantic_cost(252));
  EXPECT_FALSE(valid_max_semantic_cost(253));  // INSCRIBED
  EXPECT_FALSE(valid_max_semantic_cost(254));  // LETHAL
  EXPECT_FALSE(valid_max_semantic_cost(255));  // NO_INFORMATION
  EXPECT_FALSE(valid_max_semantic_cost(1000));
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
// = 8 per quadrant, 32 core cells (painted max_cost 200 under D-26, was 254).
TEST(PaintDisc, PersonCoreCount) {
  Grid g(0);
  paint_disc(g.view, 0.5, 0.5, kPersonCore);
  EXPECT_EQ(g.count(200), 32);
  EXPECT_EQ(g.count(254), 0);
  EXPECT_EQ(g.count(0), 100 - 32);
}

TEST(PaintDisc, NeverPaintsLethalOrInscribed) {
  Grid g(0);
  paint_disc(g.view, 0.5, 0.5, kPerson);
  EXPECT_EQ(g.count(254), 0);
  EXPECT_EQ(g.count(253), 0);
  // Only the 32 core cells reach 200: the closest ring cells, e.g. offset (0.35, 0.05) or
  // (0.25, 0.25), have d = 0.3536, d - r = 0.0036: 200 * exp(-0.0107) = 197.87 -> 198.
  EXPECT_EQ(g.count(200), 32);
  EXPECT_EQ(g.count(0), 0);  // whole 1 m x 1 m grid is within r + 1.0 of the centre
  // Corner cell (0, 0): centre (0.05, 0.05), d = 0.6364, d - r = 0.2864:
  // was 252 * exp(-0.8592) = 106.71 -> 107; now 200 * exp(-0.8592) = 84.70 -> 85.
  EXPECT_EQ(g.at(0, 0), 85);
}

TEST(PaintDisc, NeverLowersHigherExistingCost) {
  // Existing 253 everywhere: every semantic cost (<= 200) is lower, so nothing changes.
  // (Under 7.3 the 32 core cells became 254; under D-26 they stay 253.)
  Grid g(253);
  paint_disc(g.view, 0.5, 0.5, kPerson);
  EXPECT_EQ(g.count(254), 0);
  EXPECT_EQ(g.count(253), 100);
}

TEST(PaintDisc, PreservesLidarLethalCells) {
  // LiDAR lethal cells (254) inside the core and inside the ring survive std::max untouched,
  // while the rest of the core takes max_cost.
  Grid g(0);
  const int core_i = 4;  // centre (0.45, 0.45), d = 0.0707 -> core
  const int ring_i = 0;  // cell (0, 0), d = 0.6364 -> ring (85)
  g.cells[static_cast<std::size_t>(core_i * 10 + core_i)] = 254;
  g.cells[static_cast<std::size_t>(ring_i)] = 254;
  paint_disc(g.view, 0.5, 0.5, kPerson);
  EXPECT_EQ(g.at(core_i, core_i), 254);
  EXPECT_EQ(g.at(ring_i, ring_i), 254);
  EXPECT_EQ(g.count(254), 2);
  EXPECT_EQ(g.count(200), 32 - 1);  // one core cell is held at 254 by LiDAR
}

TEST(PaintDisc, ClippedAtGridBorder) {
  // Centre on the grid corner: only one quadrant (8 cells) of the core lies inside.
  Grid g(0);
  paint_disc(g.view, 0.0, 0.0, kPersonCore);
  EXPECT_EQ(g.count(200), 8);
  // Entirely outside: untouched.
  Grid h(0);
  paint_disc(h.view, -5.0, -5.0, kPerson);
  EXPECT_EQ(h.count(0), 100);
}

TEST(PaintDisc, ClippedToUpdateWindow) {
  // Only columns i in [0, 5) may be written: half of the 32 core cells.
  Grid g(0);
  paint_disc(g.view, 0.5, 0.5, kPersonCore, CellWindow{0, 0, 5, 10});
  EXPECT_EQ(g.count(200), 16);
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
  const ClassCostParams chair{0.25, 0.3, 3.0, 200};
  paint_disc(g.view, 0.5, 0.5, chair);
  // Cell (4, 4): centre (0.45, 0.45), d = 0.0707 -> core, max_cost (was 254).
  EXPECT_EQ(g.at(4, 4), 200);
  // Cell (6, 4): centre (0.65, 0.45), d = 0.1581 -> core, max_cost (was 254).
  EXPECT_EQ(g.at(6, 4), 200);
  // Cell (7, 4): centre (0.75, 0.45), d = 0.2550, d - r = 0.0050:
  // was 252 * exp(-0.0150) = 248.25 -> 248; now 200 * exp(-0.0150) = 197.02 -> 197.
  EXPECT_EQ(g.at(7, 4), 197);
  // Cell (0, 0): centre (0.05, 0.05), d = 0.6364 > 0.55 -> unknown kept.
  EXPECT_EQ(g.at(0, 0), 255);
}
