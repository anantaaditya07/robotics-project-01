// Unit tests for tracker.hpp (architecture 7.2, "Tracking"). Expected values are
// hand-computed; stamps and coordinates are chosen to be exact in binary where
// equality matters.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "semnav_perception/tracker.hpp"

namespace sp = semnav_perception;

namespace {

constexpr double kTol = 1e-12;

sp::TrackerParams makeParams(double gate = 0.6, double alpha = 0.5, double ttl = 2.0) {
  sp::TrackerParams p;
  p.assoc_gate = gate;
  p.smoothing_alpha = alpha;
  p.ttl = ttl;
  return p;
}

sp::Observation obs(const std::string& cls, double x, double y, double conf = 0.9,
                    double range = 1.0) {
  sp::Observation o;
  o.class_name = cls;
  o.x = x;
  o.y = y;
  o.confidence = conf;
  o.range = range;
  return o;
}

const sp::Track* findById(const std::vector<sp::Track>& tracks, uint32_t id) {
  for (const auto& t : tracks) {
    if (t.id == id) {
      return &t;
    }
  }
  return nullptr;
}

}  // namespace

TEST(TrackerParams, DefaultsMatchDocumentedValues) {
  const sp::TrackerParams p;
  EXPECT_DOUBLE_EQ(p.assoc_gate, 0.6);
  EXPECT_DOUBLE_EQ(p.smoothing_alpha, 0.5);
  EXPECT_DOUBLE_EQ(p.ttl, 2.0);
  EXPECT_NO_THROW(sp::Tracker{p});
}

TEST(TrackerParams, BadParamsThrow) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(sp::Tracker(makeParams(0.0, 0.5, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(-0.1, 0.5, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(nan, 0.5, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(inf, 0.5, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, 0.0, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, 1.01, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, nan, 2.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, 0.5, 0.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, 0.5, -1.0)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, 0.5, nan)), std::invalid_argument);
  EXPECT_THROW(sp::Tracker(makeParams(0.6, 0.5, inf)), std::invalid_argument);
  // alpha == 1 is the inclusive upper bound (no smoothing).
  EXPECT_NO_THROW(sp::Tracker(makeParams(0.6, 1.0, 2.0)));
}

TEST(Tracker, SingleObjectKeepsStableId) {
  sp::Tracker tr(makeParams());
  auto out = tr.update({obs("person", 1.0, 2.0, 0.8, 3.0)}, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_EQ(out[0].class_name, "person");
  EXPECT_EQ(out[0].hits, 1u);
  EXPECT_DOUBLE_EQ(out[0].first_seen, 0.0);
  for (int i = 1; i <= 5; ++i) {
    out = tr.update({obs("person", 1.0 + 0.1 * i, 2.0, 0.7, 2.5)}, 0.1 * i);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 1u);
  }
  EXPECT_EQ(out[0].hits, 6u);
  EXPECT_EQ(out[0].misses, 0u);
  EXPECT_DOUBLE_EQ(out[0].first_seen, 0.0);
  EXPECT_DOUBLE_EQ(out[0].last_seen, 0.5);
  // confidence and range are the latest observation, not smoothed.
  EXPECT_DOUBLE_EQ(out[0].confidence, 0.7);
  EXPECT_DOUBLE_EQ(out[0].range, 2.5);
  EXPECT_EQ(tr.tracks().size(), 1u);
}

TEST(Tracker, ExponentialSmoothingMath) {
  sp::Tracker tr(makeParams(0.6, 0.5, 2.0));
  tr.update({obs("person", 0.0, 0.0)}, 0.0);
  auto out = tr.update({obs("person", 0.4, -0.4)}, 0.1);  // 0.5*0.4 + 0.5*0 = 0.2
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 0.2, kTol);
  EXPECT_NEAR(out[0].y, -0.2, kTol);

  // Spec example: 0 then 1 -> 0.5, then 1 -> 0.75 (gate widened so 1 m associates).
  sp::Tracker tr2(makeParams(2.0, 0.5, 2.0));
  tr2.update({obs("chair", 0.0, 0.0)}, 0.0);
  out = tr2.update({obs("chair", 1.0, 1.0)}, 0.1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, 0.5, kTol);
  EXPECT_NEAR(out[0].y, 0.5, kTol);
  out = tr2.update({obs("chair", 1.0, 1.0)}, 0.2);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_NEAR(out[0].x, 0.75, kTol);
  EXPECT_NEAR(out[0].y, 0.75, kTol);

  // alpha = 1 means no smoothing.
  sp::Tracker tr3(makeParams(2.0, 1.0, 2.0));
  tr3.update({obs("chair", 0.0, 0.0)}, 0.0);
  out = tr3.update({obs("chair", 1.0, 0.0)}, 0.1);
  EXPECT_NEAR(out[0].x, 1.0, kTol);
}

TEST(Tracker, DistanceGate) {
  {
    sp::Tracker tr(makeParams());
    tr.update({obs("person", 0.0, 0.0)}, 0.0);
    auto out = tr.update({obs("person", 0.59, 0.0)}, 0.1);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 1u);
    EXPECT_EQ(out[0].hits, 2u);
  }
  {
    sp::Tracker tr(makeParams());
    tr.update({obs("person", 0.0, 0.0)}, 0.0);
    auto out = tr.update({obs("person", 0.61, 0.0)}, 0.1);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].id, 1u);
    EXPECT_EQ(out[0].misses, 1u);
    EXPECT_NEAR(out[0].x, 0.0, kTol);
    EXPECT_EQ(out[1].id, 2u);
    EXPECT_NEAR(out[1].x, 0.61, kTol);
  }
  {
    // Gate is Euclidean, not per-axis: (0.36, 0.47) is ~0.592 m away -> associates.
    sp::Tracker tr(makeParams(0.6, 0.5, 2.0));
    tr.update({obs("person", 0.0, 0.0)}, 0.0);
    auto out = tr.update({obs("person", 0.36, 0.47)}, 0.1);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 1u);
  }
}

TEST(Tracker, DifferentClassesNeverAssociate) {
  sp::Tracker tr(makeParams());
  auto out = tr.update({obs("person", 1.0, 1.0), obs("chair", 1.0, 1.0)}, 0.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_EQ(out[0].class_name, "person");
  EXPECT_EQ(out[1].id, 2u);
  EXPECT_EQ(out[1].class_name, "chair");

  // Same spot again, listed in the other order: each class keeps its id.
  out = tr.update({obs("chair", 1.0, 1.0), obs("person", 1.0, 1.0)}, 0.1);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].class_name, "person");
  EXPECT_EQ(out[0].hits, 2u);
  EXPECT_EQ(out[1].class_name, "chair");
  EXPECT_EQ(out[1].hits, 2u);

  // Only a chair right on top of the person track: the person misses, and a
  // chair track is matched rather than the person track.
  out = tr.update({obs("chair", 1.0, 1.0)}, 0.2);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].misses, 1u);
  EXPECT_EQ(out[1].misses, 0u);
  EXPECT_EQ(out[1].hits, 3u);
}

TEST(Tracker, GlobalGreedyPicksNearestPairFirst) {
  // Tracks A=(0,0) id1, B=(0.5,0) id2. Observations o1=(0.35,0), o2=(0.9,0).
  // Gated pairs: B-o1 0.15, A-o1 0.35, B-o2 0.40 (A-o2 0.9 is out of gate).
  // Greedy accepts B-o1 first; A-o1 and B-o2 are then blocked. So A misses and
  // o2 spawns id 3. (Track-ordered NN would give A-o1, B-o2; Hungarian would
  // too. This test pins the documented greedy behaviour.)
  sp::Tracker tr(makeParams(0.6, 1.0, 2.0));
  tr.update({obs("person", 0.0, 0.0), obs("person", 0.5, 0.0)}, 0.0);
  auto out = tr.update({obs("person", 0.35, 0.0), obs("person", 0.9, 0.0)}, 0.1);
  ASSERT_EQ(out.size(), 3u);
  const sp::Track* a = findById(out, 1);
  const sp::Track* b = findById(out, 2);
  const sp::Track* c = findById(out, 3);
  ASSERT_TRUE(a && b && c);
  EXPECT_EQ(a->misses, 1u);
  EXPECT_NEAR(a->x, 0.0, kTol);
  EXPECT_EQ(b->misses, 0u);
  EXPECT_NEAR(b->x, 0.35, kTol);
  EXPECT_NEAR(c->x, 0.9, kTol);
}

TEST(Tracker, TwoCloseObjectsKeepIdsWhenListOrderSwaps) {
  // Two people 0.4 m apart (inside each other's gate) walking in parallel.
  // The observation list order flips every frame; nearest-first must keep ids.
  sp::Tracker tr(makeParams(0.6, 1.0, 2.0));
  tr.update({obs("person", 0.0, 0.0), obs("person", 0.0, 0.4)}, 0.0);
  for (int i = 1; i <= 6; ++i) {
    const double x = 0.1 * i;
    std::vector<sp::Observation> o = {obs("person", x, 0.0), obs("person", x, 0.4)};
    if (i % 2 == 1) {
      std::swap(o[0], o[1]);
    }
    auto out = tr.update(o, 0.1 * i);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_NEAR(findById(out, 1)->y, 0.0, kTol);
    EXPECT_NEAR(findById(out, 2)->y, 0.4, kTol);
    EXPECT_NEAR(findById(out, 1)->x, x, kTol);
  }
}

TEST(Tracker, TtlBoundary) {
  // ttl = 2: now - last_seen == 2 keeps the track, > 2 expires it.
  sp::Tracker tr(makeParams(0.6, 0.5, 2.0));
  tr.update({obs("person", 0.0, 0.0)}, 1.0);
  auto out = tr.update({}, 3.0);  // exactly 2.0 s
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].misses, 1u);
  out = tr.update({}, 3.25);  // 2.25 s
  EXPECT_TRUE(out.empty());
  EXPECT_TRUE(tr.tracks().empty());

  // Standalone expire() follows the same rule.
  sp::Tracker tr2(makeParams(0.6, 0.5, 2.0));
  tr2.update({obs("chair", 0.0, 0.0)}, 1.0);
  tr2.expire(3.0);
  EXPECT_EQ(tr2.tracks().size(), 1u);
  tr2.expire(3.0625);
  EXPECT_TRUE(tr2.tracks().empty());
}

TEST(Tracker, ExpiryRunsBeforeAssociation) {
  // A track that is stale at this stamp is removed first, so an observation at
  // the same spot starts a new id instead of reviving the ghost.
  sp::Tracker tr(makeParams(0.6, 0.5, 2.0));
  tr.update({obs("person", 0.0, 0.0)}, 0.0);
  auto out = tr.update({obs("person", 0.0, 0.0)}, 2.5);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 2u);
  EXPECT_DOUBLE_EQ(out[0].first_seen, 2.5);
}

TEST(Tracker, IdsNotReusedAfterExpiry) {
  sp::Tracker tr(makeParams());
  tr.update({obs("person", 0.0, 0.0), obs("chair", 5.0, 5.0)}, 0.0);  // ids 1, 2
  auto out = tr.update({}, 10.0);
  EXPECT_TRUE(out.empty());
  out = tr.update({obs("person", 0.0, 0.0)}, 10.5);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 3u);
}

TEST(Tracker, MissCounterIncrementsAndResets) {
  sp::Tracker tr(makeParams());
  tr.update({obs("person", 0.0, 0.0)}, 0.0);
  auto out = tr.update({}, 0.1);
  EXPECT_EQ(out[0].misses, 1u);
  out = tr.update({obs("person", 3.0, 3.0)}, 0.2);  // far away: still a miss for id 1
  EXPECT_EQ(findById(out, 1)->misses, 2u);
  EXPECT_EQ(findById(out, 2)->misses, 0u);
  out = tr.update({}, 0.3);
  EXPECT_EQ(findById(out, 1)->misses, 3u);
  EXPECT_EQ(findById(out, 2)->misses, 1u);
  out = tr.update({obs("person", 0.1, 0.0)}, 0.4);
  EXPECT_EQ(findById(out, 1)->misses, 0u);
  EXPECT_EQ(findById(out, 1)->hits, 2u);
  EXPECT_EQ(findById(out, 2)->misses, 2u);
  EXPECT_DOUBLE_EQ(findById(out, 1)->last_seen, 0.4);
  EXPECT_DOUBLE_EQ(findById(out, 2)->last_seen, 0.2);
}

TEST(Tracker, OutOfOrderStampDoesNotRewindLastSeen) {
  sp::Tracker tr(makeParams(0.6, 0.5, 2.0));
  tr.update({obs("person", 0.0, 0.0)}, 5.0);
  auto out = tr.update({obs("person", 0.5, 0.0)}, 4.75);  // older stamp
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_NEAR(out[0].x, 0.25, kTol);  // position still smoothed
  EXPECT_DOUBLE_EQ(out[0].last_seen, 5.0);
  EXPECT_DOUBLE_EQ(out[0].first_seen, 5.0);
  EXPECT_EQ(out[0].hits, 2u);
  // TTL measured from the max stamp: 7.0 - 5.0 == ttl keeps it.
  out = tr.update({}, 7.0);
  EXPECT_EQ(out.size(), 1u);
}

TEST(Tracker, ResetClearsTracksButIdsContinue) {
  sp::Tracker tr(makeParams());
  tr.update({obs("person", 0.0, 0.0), obs("person", 2.0, 0.0)}, 0.0);  // ids 1, 2
  tr.reset();
  EXPECT_TRUE(tr.tracks().empty());
  auto out = tr.update({obs("person", 0.0, 0.0)}, 0.1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 3u);
  EXPECT_EQ(out[0].hits, 1u);
}

TEST(Tracker, OutputSortedByIdAndMatchesTracks) {
  sp::Tracker tr(makeParams());
  tr.update({obs("a", 0.0, 0.0), obs("b", 1.0, 0.0), obs("c", 2.0, 0.0)}, 0.0);
  tr.update({obs("b", 1.0, 0.0)}, 1.5);
  auto out = tr.update({obs("d", 3.0, 0.0), obs("a", 9.0, 9.0)}, 2.25);  // a, c expire
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].id, 2u);  // b
  EXPECT_EQ(out[1].id, 4u);  // d
  EXPECT_EQ(out[2].id, 5u);  // new a
  ASSERT_EQ(tr.tracks().size(), out.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_EQ(tr.tracks()[i].id, out[i].id);
  }
}

TEST(Tracker, NonFiniteObservationsIgnored) {
  sp::Tracker tr(makeParams());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto out = tr.update({obs("person", nan, 0.0), obs("person", 0.0, 0.0)}, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  out = tr.update({obs("person", 0.0, std::numeric_limits<double>::infinity())}, 0.1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].misses, 1u);
  EXPECT_NEAR(out[0].x, 0.0, kTol);
}
