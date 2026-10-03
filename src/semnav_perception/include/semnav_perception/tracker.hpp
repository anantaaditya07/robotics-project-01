// Multi-object tracker for semantic obstacles (architecture 7.2, "Tracking").
//
// "Nearest-neighbour association by class and distance gate (0.6 m), exponential
// smoothing of position, id assignment, miss counter; obstacle expires after ttl
// (default 2 s) so ghosts vanish." -- expiry replaced by negative evidence (D-25), see below.
//
// ROS-free, header-only, plain STL. All positions are in the map frame, metres;
// all stamps are seconds (the node converts rclcpp::Time to double).
//
// Behaviour summary (each point is covered by test_tracker.cpp):
//  * Association is per class only: an observation can only match a track with
//    an identical class_name.
//  * Within a class the association is GLOBAL-GREEDY nearest neighbour, not
//    Hungarian: every (track, observation) pair with distance <= assoc_gate is
//    collected, pairs are sorted by distance (ties broken by track id, then
//    observation index, so results are deterministic), and pairs are accepted
//    in that order if neither side is already used. This is optimal for the
//    common case of well-separated objects and is O(P log P) in the number of
//    gated pairs; it can be sub-optimal in total cost for crowded scenes.
//  * Gate test is inclusive: distance <= assoc_gate associates.
//  * Matched track: x,y = alpha * obs + (1 - alpha) * old; confidence and range
//    are taken from the latest observation (not smoothed); last_seen =
//    max(last_seen, stamp); hits++; misses = 0.
//  * Unmatched track (survived expiry, but no observation in this update):
//    misses++ only if the caller's `observable` predicate says so (in view, in
//    range, frame fused); a track reaching miss_frames misses is removed (D-25).
//    Without a predicate no update counts as a miss.
//  * Unmatched observation: a new track with the next id. Ids start at 1,
//    increase monotonically and are never reused, not even after reset().
//    uint32 wrap-around (4e9 tracks) is not a concern for this application.
//  * Observations with non-finite x or y are ignored (they cannot be gated).
//  * Age expiry: a track is removed when now - last_seen > max_age (strictly
//    greater; == max_age keeps the track). update() runs expire(stamp)
//    BEFORE association, so a track that has already timed out cannot be
//    revived by a late observation; that observation spawns a new id instead.
//  * Out-of-order stamps: last_seen never moves backwards. A match with a stamp
//    older than last_seen still smooths the position but keeps last_seen.
//    first_seen of a track is the stamp of the observation that created it.
//  * tracks() and the result of update() are always sorted by ascending id.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace semnav_perception {

/// Tunables; the node fills these from ROS parameters (assoc_gate,
/// smoothing_alpha, miss_frames, max_age). The in-struct values are only documented defaults.
///
/// Expiry (D-25, replaces the pure 2 s TTL of architecture 7.2): negative evidence only.
/// A track is removed when it was OBSERVABLE (the caller's predicate: inside the camera view and
/// LiDAR range, in a frame that was actually fused) and unmatched for `miss_frames` consecutive
/// observable updates. Updates where it is not observable neither count nor reset its misses.
/// Independently, any track not matched for more than `max_age` seconds is removed.
struct TrackerParams {
  /// Max distance [m] between a track and an observation of the same class for
  /// them to be associated. Default 0.6 m per architecture 7.2.
  double assoc_gate{0.6};
  /// Exponential smoothing weight of the NEW observation, in (0, 1]. 1.0
  /// disables smoothing. The PDF gives no value; 0.5 is our documented default.
  double smoothing_alpha{0.5};
  /// Consecutive observable-but-unmatched updates after which a track is dropped (>= 1).
  uint32_t miss_frames{10};
  /// Time [s] since last_seen after which a track is dropped regardless of visibility.
  double max_age{120.0};
  /// D-26: a track is CONFIRMED once it has absorbed at least min_hits observations (>= 1).
  /// Callers publish confirmed tracks only.
  uint32_t min_hits{3};
  /// D-26: unconfirmed (tentative) tracks are dropped after this many seconds unseen.
  double tentative_max_age{2.0};
  /// D-26: same-class tracks closer than this [m] are merged (the node uses the class footprint
  /// diameter, 2 * class_radius). Key "default" applies to classes not listed; missing or <= 0
  /// disables merging for that class.
  std::map<std::string, double> merge_distance{};
};

/// Throws std::invalid_argument unless assoc_gate > 0 (finite),
/// 0 < smoothing_alpha <= 1, miss_frames >= 1 and max_age > 0 (finite). NaN is rejected.
inline void validate(const TrackerParams& p) {
  if (!(p.assoc_gate > 0.0) || !std::isfinite(p.assoc_gate)) {
    throw std::invalid_argument("TrackerParams: assoc_gate must be finite and > 0");
  }
  if (!(p.smoothing_alpha > 0.0) || !(p.smoothing_alpha <= 1.0)) {
    throw std::invalid_argument("TrackerParams: smoothing_alpha must be in (0, 1]");
  }
  if (p.miss_frames < 1) {
    throw std::invalid_argument("TrackerParams: miss_frames must be >= 1");
  }
  if (!(p.max_age > 0.0) || !std::isfinite(p.max_age)) {
    throw std::invalid_argument("TrackerParams: max_age must be finite and > 0");
  }
  if (p.min_hits < 1) {
    throw std::invalid_argument("TrackerParams: min_hits must be >= 1");
  }
  if (!(p.tentative_max_age > 0.0) || !std::isfinite(p.tentative_max_age)) {
    throw std::invalid_argument("TrackerParams: tentative_max_age must be finite and > 0");
  }
  for (const auto& [cls, d] : p.merge_distance) {
    if (!std::isfinite(d)) {
      throw std::invalid_argument("TrackerParams: merge_distance for " + cls + " must be finite");
    }
  }
}

/// One localised detection in the map frame.
struct Observation {
  std::string class_name;
  double x{0.0};           ///< map frame [m]
  double y{0.0};           ///< map frame [m]
  double confidence{0.0};  ///< detector score
  double range{0.0};       ///< sensor-to-object distance [m]
};

/// One tracked semantic obstacle.
struct Track {
  uint32_t id{0};
  std::string class_name;
  double x{0.0};           ///< smoothed, map frame [m]
  double y{0.0};           ///< smoothed, map frame [m]
  double confidence{0.0};  ///< latest observation
  double range{0.0};       ///< latest observation [m]
  double first_seen{0.0};  ///< [s]
  double last_seen{0.0};   ///< [s], monotonic non-decreasing
  uint32_t hits{0};        ///< number of observations absorbed (1 at creation)
  uint32_t misses{0};      ///< consecutive OBSERVABLE updates without a match
};

/// Returns true if an unmatched track counts as negative evidence in this update (D-25).
using ObservableFn = std::function<bool(const Track&)>;

class Tracker {
 public:
  explicit Tracker(TrackerParams params) : params_(params) { validate(params_); }

  const TrackerParams& params() const { return params_; }

  /// Expire stale tracks at `stamp_sec`, associate `observations`, update or
  /// create tracks, and return the current tracks sorted by id.
  /// `observable` (may be empty) says, per track, whether a miss in this update is negative
  /// evidence. Empty = no track is observable (only max_age can remove tracks).
  std::vector<Track> update(const std::vector<Observation>& observations, double stamp_sec,
                            const ObservableFn& observable = {}) {
    expire(stamp_sec);

    // Gated candidate pairs: (squared distance, track index, observation index).
    // Track indices follow id order, so the tuple sort breaks ties by id.
    using Pair = std::tuple<double, std::size_t, std::size_t>;
    std::vector<Pair> pairs;
    const double gate_sq = params_.assoc_gate * params_.assoc_gate;
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      for (std::size_t oi = 0; oi < observations.size(); ++oi) {
        const Observation& o = observations[oi];
        if (!finite_xy(o) || o.class_name != tracks_[ti].class_name) {
          continue;
        }
        const double dx = o.x - tracks_[ti].x;
        const double dy = o.y - tracks_[ti].y;
        const double d_sq = dx * dx + dy * dy;
        if (d_sq <= gate_sq) {
          pairs.emplace_back(d_sq, ti, oi);
        }
      }
    }
    std::sort(pairs.begin(), pairs.end());

    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<bool> obs_used(observations.size(), false);
    const double a = params_.smoothing_alpha;
    for (const auto& [d_sq, ti, oi] : pairs) {
      (void)d_sq;
      if (track_used[ti] || obs_used[oi]) {
        continue;
      }
      track_used[ti] = true;
      obs_used[oi] = true;
      Track& t = tracks_[ti];
      const Observation& o = observations[oi];
      t.x = a * o.x + (1.0 - a) * t.x;
      t.y = a * o.y + (1.0 - a) * t.y;
      t.confidence = o.confidence;
      t.range = o.range;
      t.last_seen = std::max(t.last_seen, stamp_sec);
      ++t.hits;
      t.misses = 0;
    }

    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (!track_used[ti] && observable && observable(tracks_[ti])) {
        ++tracks_[ti].misses;
      }
    }
    const uint32_t miss_limit = params_.miss_frames;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [miss_limit](const Track& t) { return t.misses >= miss_limit; }),
                  tracks_.end());

    // New tracks get increasing ids and are appended, so tracks_ stays sorted.
    for (std::size_t oi = 0; oi < observations.size(); ++oi) {
      const Observation& o = observations[oi];
      if (obs_used[oi] || !finite_xy(o)) {
        continue;
      }
      Track t;
      t.id = next_id_++;
      t.class_name = o.class_name;
      t.x = o.x;
      t.y = o.y;
      t.confidence = o.confidence;
      t.range = o.range;
      t.first_seen = stamp_sec;
      t.last_seen = stamp_sec;
      t.hits = 1;
      t.misses = 0;
      tracks_.push_back(std::move(t));
    }

    merge_close_tracks();  // D-26, after new tracks so a fresh ghost merges immediately
    return tracks_;
  }

  /// Current tracks, sorted by ascending id.
  const std::vector<Track>& tracks() const { return tracks_; }

  /// D-26: confirmed tracks only (hits >= min_hits), sorted by id. Publish these.
  std::vector<Track> confirmed() const {
    std::vector<Track> out;
    for (const Track& t : tracks_) {
      if (is_confirmed(t)) {
        out.push_back(t);
      }
    }
    return out;
  }

  bool is_confirmed(const Track& t) const { return t.hits >= params_.min_hits; }

  /// Remove every track with now_sec - last_seen > max_age.
  void expire(double now_sec) {
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this, now_sec](const Track& t) {
                                   const double age = is_confirmed(t) ? params_.max_age
                                                                      : params_.tentative_max_age;
                                   return now_sec - t.last_seen > age;
                                 }),
                  tracks_.end());
  }

  /// Drop all tracks. The id counter is NOT reset, so ids issued after reset()
  /// never collide with ids a subscriber has already seen (e.g. RViz markers).
  void reset() { tracks_.clear(); }

 private:
  double merge_distance_for(const std::string& cls) const {
    auto it = params_.merge_distance.find(cls);
    if (it == params_.merge_distance.end()) {
      it = params_.merge_distance.find("default");
    }
    return it == params_.merge_distance.end() ? 0.0 : it->second;
  }

  /// D-26: merge same-class tracks closer than merge_distance_for(class). The survivor is the
  /// track with more hits (tie: lower id); its position becomes the hits-weighted mean, hits are
  /// summed, last_seen = max, first_seen = min, misses = min, confidence/range from the more
  /// recently seen one. Repeats until no pair is closer than the distance.
  void merge_close_tracks() {
    bool merged = true;
    while (merged) {
      merged = false;
      for (std::size_t i = 0; i < tracks_.size() && !merged; ++i) {
        for (std::size_t j = i + 1; j < tracks_.size() && !merged; ++j) {
          Track& a = tracks_[i];
          Track& b = tracks_[j];
          const double md = merge_distance_for(a.class_name);
          if (a.class_name != b.class_name || !(md > 0.0) ||
              std::hypot(a.x - b.x, a.y - b.y) >= md) {
            continue;
          }
          const bool keep_a = a.hits >= b.hits;  // a has the lower id (tracks_ sorted by id)
          Track& keep = keep_a ? a : b;
          const Track drop = keep_a ? b : a;
          const double wk = static_cast<double>(keep.hits);
          const double wd = static_cast<double>(drop.hits);
          keep.x = (wk * keep.x + wd * drop.x) / (wk + wd);
          keep.y = (wk * keep.y + wd * drop.y) / (wk + wd);
          if (drop.last_seen > keep.last_seen) {
            keep.confidence = drop.confidence;
            keep.range = drop.range;
          }
          keep.last_seen = std::max(keep.last_seen, drop.last_seen);
          keep.first_seen = std::min(keep.first_seen, drop.first_seen);
          keep.misses = std::min(keep.misses, drop.misses);
          keep.hits += drop.hits;
          tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(keep_a ? j : i));
          merged = true;
        }
      }
    }
  }

  static bool finite_xy(const Observation& o) { return std::isfinite(o.x) && std::isfinite(o.y); }

  TrackerParams params_;
  std::vector<Track> tracks_;
  uint32_t next_id_{1};
};

}  // namespace semnav_perception
