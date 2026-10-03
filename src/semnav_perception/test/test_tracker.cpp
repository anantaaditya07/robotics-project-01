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

sp::TrackerParams makeParams(double gate = 0.6, double alpha = 0.5, double max_age = 2.0) {
  sp::TrackerParams p;
  p.assoc_gate = gate;
  p.smoothing_alpha = alpha;
  p.max_age = max_age;
  p.min_hits = 1;  // every track confirmed: tests below predate D-26 confirmation
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
  EXPECT_DOUBLE_EQ(p.max_age, 120.0);
  EXPECT_EQ(p.min_hits, 3U);
  EXPECT_DOUBLE_EQ(p.tentative_max_age, 2.0);
  EXPECT_TRUE(p.merge_distance.empty());
  EXPECT_EQ(p.miss_frames, 10U);
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
  // Every unmatched update counts as a miss here (pre-D-25 semantics for the miss checks).
  const auto all = [](const sp::Track&) { return true; };
  {
    sp::Tracker tr(makeParams());
    tr.update({obs("person", 0.0, 0.0)}, 0.0, all);
    auto out = tr.update({obs("person", 0.59, 0.0)}, 0.1, all);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 1u);
    EXPECT_EQ(out[0].hits, 2u);
  }
  {
    sp::Tracker tr(makeParams());
    tr.update({obs("person", 0.0, 0.0)}, 0.0, all);
    auto out = tr.update({obs("person", 0.61, 0.0)}, 0.1, all);
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
    tr.update({obs("person", 0.0, 0.0)}, 0.0, all);
    auto out = tr.update({obs("person", 0.36, 0.47)}, 0.1, all);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].id, 1u);
  }
}

TEST(Tracker, DifferentClassesNeverAssociate) {
  // Every unmatched update counts as a miss here (pre-D-25 semantics for the miss checks).
  const auto all = [](const sp::Track&) { return true; };
  sp::Tracker tr(makeParams());
  auto out = tr.update({obs("person", 1.0, 1.0), obs("chair", 1.0, 1.0)}, 0.0, all);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_EQ(out[0].class_name, "person");
  EXPECT_EQ(out[1].id, 2u);
  EXPECT_EQ(out[1].class_name, "chair");

  // Same spot again, listed in the other order: each class keeps its id.
  out = tr.update({obs("chair", 1.0, 1.0), obs("person", 1.0, 1.0)}, 0.1, all);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].class_name, "person");
  EXPECT_EQ(out[0].hits, 2u);
  EXPECT_EQ(out[1].class_name, "chair");
  EXPECT_EQ(out[1].hits, 2u);

  // Only a chair right on top of the person track: the person misses, and a
  // chair track is matched rather than the person track.
  out = tr.update({obs("chair", 1.0, 1.0)}, 0.2, all);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].misses, 1u);
  EXPECT_EQ(out[1].misses, 0u);
  EXPECT_EQ(out[1].hits, 3u);
}

TEST(Tracker, GlobalGreedyPicksNearestPairFirst) {
  // Every unmatched update counts as a miss here (pre-D-25 semantics for the miss checks).
  const auto all = [](const sp::Track&) { return true; };
  // Tracks A=(0,0) id1, B=(0.5,0) id2. Observations o1=(0.35,0), o2=(0.9,0).
  // Gated pairs: B-o1 0.15, A-o1 0.35, B-o2 0.40 (A-o2 0.9 is out of gate).
  // Greedy accepts B-o1 first; A-o1 and B-o2 are then blocked. So A misses and
  // o2 spawns id 3. (Track-ordered NN would give A-o1, B-o2; Hungarian would
  // too. This test pins the documented greedy behaviour.)
  sp::Tracker tr(makeParams(0.6, 1.0, 2.0));
  tr.update({obs("person", 0.0, 0.0), obs("person", 0.5, 0.0)}, 0.0, all);
  auto out = tr.update({obs("person", 0.35, 0.0), obs("person", 0.9, 0.0)}, 0.1, all);
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
  // Every unmatched update counts as a miss here (pre-D-25 semantics for the miss checks).
  const auto all = [](const sp::Track&) { return true; };
  // max_age = 2: now - last_seen == 2 keeps the track, > 2 expires it.
  sp::Tracker tr(makeParams(0.6, 0.5, 2.0));
  tr.update({obs("person", 0.0, 0.0)}, 1.0, all);
  auto out = tr.update({}, 3.0, all);  // exactly 2.0 s
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].misses, 1u);
  out = tr.update({}, 3.25, all);  // 2.25 s
  EXPECT_TRUE(out.empty());
  EXPECT_TRUE(tr.tracks().empty());

  // Standalone expire() follows the same rule.
  sp::Tracker tr2(makeParams(0.6, 0.5, 2.0));
  tr2.update({obs("chair", 0.0, 0.0)}, 1.0, all);
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
  const auto all = [](const sp::Track&) { return true; };  // every miss is negative evidence
  tr.update({obs("person", 0.0, 0.0)}, 0.0);
  auto out = tr.update({}, 0.1, all);
  EXPECT_EQ(out[0].misses, 1u);
  out = tr.update({obs("person", 3.0, 3.0)}, 0.2, all);  // far away: still a miss for id 1
  EXPECT_EQ(findById(out, 1)->misses, 2u);
  EXPECT_EQ(findById(out, 2)->misses, 0u);
  out = tr.update({}, 0.3, all);
  EXPECT_EQ(findById(out, 1)->misses, 3u);
  EXPECT_EQ(findById(out, 2)->misses, 1u);
  out = tr.update({obs("person", 0.1, 0.0)}, 0.4, all);
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
  // Age measured from the max stamp: 7.0 - 5.0 == max_age keeps it.
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
  // Every unmatched update counts as a miss here (pre-D-25 semantics for the miss checks).
  const auto all = [](const sp::Track&) { return true; };
  sp::Tracker tr(makeParams());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto out = tr.update({obs("person", nan, 0.0), obs("person", 0.0, 0.0)}, 0.0, all);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  out = tr.update({obs("person", 0.0, std::numeric_limits<double>::infinity())}, 0.1, all);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].misses, 1u);
  EXPECT_NEAR(out[0].x, 0.0, kTol);
}

// ---------------------------------------------------------------- D-25 negative evidence

TEST(TrackerD25, InViewTrackExpiresAfterMissFrames) {
  // miss_frames 3: three observable empty updates remove the track; two do not.
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.miss_frames = 3;
  sp::Tracker tr(p);
  const auto in_view = [](const sp::Track&) { return true; };
  tr.update({obs("person", 1.0, 0.0)}, 0.0);
  EXPECT_EQ(tr.update({}, 0.1, in_view).size(), 1u);  // misses 1
  EXPECT_EQ(tr.update({}, 0.2, in_view).size(), 1u);  // misses 2
  EXPECT_TRUE(tr.update({}, 0.3, in_view).empty());   // misses 3 -> removed
}

TEST(TrackerD25, OutOfViewTrackPersistsUntilMaxAge) {
  // Not observable: no misses accumulate however many frames pass; only max_age (10 s) removes.
  sp::TrackerParams p = makeParams(0.6, 0.5, 10.0);
  p.miss_frames = 2;
  sp::Tracker tr(p);
  const auto out_of_view = [](const sp::Track&) { return false; };
  tr.update({obs("person", 1.0, 0.0)}, 0.0);
  for (int i = 1; i <= 100; ++i) {  // 100 frames over 10 s
    const auto out = tr.update({}, 0.1 * i, out_of_view);
    ASSERT_EQ(out.size(), 1u) << "frame " << i;
    EXPECT_EQ(out[0].misses, 0u);
  }
  EXPECT_TRUE(tr.update({}, 10.05, out_of_view).empty());  // 10.05 - 0 > max_age 10
}

TEST(TrackerD25, MaxAgeRemovesEvenWithoutPredicate) {
  sp::Tracker tr(makeParams(0.6, 0.5, 120.0));
  tr.update({obs("chair", 0.0, 0.0)}, 0.0);
  EXPECT_EQ(tr.update({}, 120.0).size(), 1u);  // == max_age keeps
  EXPECT_TRUE(tr.update({}, 120.5).empty());   // > max_age removes
}

TEST(TrackerD25, LeavingViewKeepsMissCountAndRedetectionResetsIt) {
  // Two in-view misses, then out of view (count frozen at 2), then back in view: one more miss
  // reaches miss_frames 3. A re-detection in between would reset the count to 0.
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.miss_frames = 3;
  sp::Tracker tr(p);
  bool visible = true;
  const auto pred = [&visible](const sp::Track&) { return visible; };
  tr.update({obs("person", 1.0, 0.0)}, 0.0);
  tr.update({}, 0.1, pred);
  tr.update({}, 0.2, pred);
  visible = false;
  auto out = tr.update({}, 0.3, pred);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].misses, 2u);
  out = tr.update({obs("person", 1.05, 0.0)}, 0.4, pred);  // re-detected while "out of view"
  EXPECT_EQ(out[0].misses, 0u);
  visible = true;
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(tr.update({}, 0.5 + 0.1 * i, pred).size(), 1u);
  }
  EXPECT_TRUE(tr.update({}, 0.8, pred).empty());
}

TEST(TrackerD25, PredicateIsPerTrack) {
  // Only the track the predicate marks observable accumulates misses.
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.miss_frames = 1;
  sp::Tracker tr(p);
  tr.update({obs("person", 1.0, 0.0), obs("chair", 5.0, 0.0)}, 0.0);
  const auto only_person = [](const sp::Track& t) { return t.class_name == "person"; };
  const auto out = tr.update({}, 0.1, only_person);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].class_name, "chair");
}

TEST(TrackerD25, BadMissFramesThrows) {
  sp::TrackerParams p = makeParams();
  p.miss_frames = 0;
  EXPECT_THROW(sp::Tracker{p}, std::invalid_argument);
}

// ---------------------------------------------------------------- D-26 ghost suppression

TEST(TrackerD26, ConfirmedOnlyAfterMinHits) {
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.min_hits = 3;
  sp::Tracker tr(p);
  tr.update({obs("person", 1.0, 0.0)}, 0.0);
  EXPECT_TRUE(tr.confirmed().empty());  // hits 1
  tr.update({obs("person", 1.0, 0.0)}, 0.1);
  EXPECT_TRUE(tr.confirmed().empty());  // hits 2
  tr.update({obs("person", 1.0, 0.0)}, 0.2);
  ASSERT_EQ(tr.confirmed().size(), 1u);  // hits 3
  EXPECT_EQ(tr.confirmed()[0].id, 1u);
}

TEST(TrackerD26, TentativeTrackExpiresFastConfirmedPersists) {
  // tentative_max_age 2 s, max_age 120 s: a 1-hit ghost dies after 2 s, a confirmed track stays.
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.min_hits = 2;
  sp::Tracker tr(p);
  tr.update({obs("person", 1.0, 0.0), obs("person", 5.0, 0.0)}, 0.0);
  tr.update({obs("person", 1.0, 0.0)}, 0.1);  // id 1 confirmed (2 hits), id 2 tentative
  auto out = tr.update({}, 2.0);              // id 2: 2.0 - 0.0 == 2.0 keeps
  EXPECT_EQ(out.size(), 2u);
  out = tr.update({}, 2.2);  // id 2: 2.2 > 2.0 -> dropped; id 1 kept (max_age 120)
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_EQ(tr.update({}, 100.0).size(), 1u);
}

TEST(TrackerD26, SameClassTracksCloserThanFootprintMerge) {
  // merge distance 0.7 m (person footprint diameter 2 * 0.35). Gate 0.6 m: an observation 0.65 m
  // away spawns a second track, which is then merged into the first (more hits survives).
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.merge_distance = {{"person", 0.7}};
  sp::Tracker tr(p);
  tr.update({obs("person", 0.0, 0.0)}, 0.0);
  tr.update({obs("person", 0.0, 0.0)}, 0.1);              // id 1: 2 hits at (0, 0)
  auto out = tr.update({obs("person", 0.65, 0.0)}, 0.2);  // id 2 spawned (1 hit), merged
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, 1u);
  EXPECT_EQ(out[0].hits, 3u);
  // hits-weighted mean: (2 * 0.0 + 1 * 0.65) / 3 = 0.216667
  EXPECT_NEAR(out[0].x, 0.65 / 3.0, 1e-9);
  EXPECT_DOUBLE_EQ(out[0].last_seen, 0.2);
}

TEST(TrackerD26, NoMergeAcrossClassesOrBeyondDistance) {
  sp::TrackerParams p = makeParams(0.6, 0.5, 120.0);
  p.merge_distance = {{"person", 0.7}, {"default", 0.5}};
  sp::Tracker tr(p);
  // person 0.75 m apart (>= 0.7): kept apart; chair 0.3 m from a person: different class.
  auto out =
      tr.update({obs("person", 0.0, 0.0), obs("person", 0.75, 0.0), obs("chair", 0.3, 0.0)}, 0.0);
  EXPECT_EQ(out.size(), 3u);
  // Two chairs 0.4 m apart use the "default" distance 0.5 -> merged into one.
  out = tr.update({obs("chair", 0.3, 0.0), obs("chair", 0.3, 0.4)}, 0.1);
  EXPECT_EQ(std::count_if(out.begin(), out.end(),
                          [](const sp::Track& t) { return t.class_name == "chair"; }),
            1);
}

TEST(TrackerD26, BadParamsThrow) {
  sp::TrackerParams p = makeParams();
  p.min_hits = 0;
  EXPECT_THROW(sp::Tracker{p}, std::invalid_argument);
  p = makeParams();
  p.tentative_max_age = 0.0;
  EXPECT_THROW(sp::Tracker{p}, std::invalid_argument);
  p = makeParams();
  p.merge_distance = {{"person", std::numeric_limits<double>::infinity()}};
  EXPECT_THROW(sp::Tracker{p}, std::invalid_argument);
}
