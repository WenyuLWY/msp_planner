/**
 * @file low_density_point_guard.h
 * @brief Give-up policy for externally published low-density points.
 *
 * The low-density points are produced and refreshed by an external node: once a
 * region is re-scanned, its representative points are updated and the old ones
 * disappear. This class therefore does NOT decide whether a point has been
 * observed. It only decides whether the planner should keep paying for it.
 *
 * A cluster that survives a planning cycle in which we already sent the robot
 * to cover it is treated as a failed attempt. Failed attempts trigger
 * exponential-backoff cooling and finally permanent abandonment, so a handful
 * of scattered, unreachable points cannot hold the robot in place forever.
 *
 * Identity is anchored to a coarse spatial cell rather than to the points
 * themselves, so replacing the representative points of the same region does
 * not reset the attempt counter.
 *
 * Integration (four call sites, independent of whether low-density points are
 * merged into the surface channel or kept as a third parallel channel):
 *   1. PlanningEnv / subscriber: once per cycle, call Update() with the newly
 *      published cloud. Only the returned targets may contribute gain.
 *   2. ViewpointManager / GetUncoveredArea: build the viewpoint -> target
 *      inverted index using only those targets, multiplied by target.weight.
 *   3. LocalCoveragePlanner: after the viewpoint set is fixed, collect the
 *      cluster_key of every low-density target covered by a selected viewpoint
 *      and call NoteServiced().
 *   4. IMPORTANT: low-density viewpoints must NOT affect exploration
 *      completion. Append them AFTER the local_coverage_complete_ check in
 *      SolveLocalCoverageProblem(), and disable this channel while returning
 *      home (the same way SetUseFrontier(false) is used). Otherwise the robot
 *      can never declare local coverage complete and never returns home
 *      (see sensor_coverage_planner_ground.cpp:1346).
 *
 * This header is dependency-free (standard library only) so it can be unit
 * tested without ROS/PCL.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace low_density_point_ns
{
struct Point3D
{
  double x;
  double y;
  double z;
  Point3D() : x(0.0), y(0.0), z(0.0)
  {
  }
  Point3D(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in)
  {
  }
};

struct LowDensityGuardParameters
{
  // Identity: coarse anchor cell. A region keeps its identity even when the
  // external publisher replaces its representative points. Should be larger
  // than the typical drift of a representative point between two scans.
  double anchor_cell_size;

  // Fine voxel used to collapse near-duplicate points before counting.
  double dedup_voxel_size;

  // Scattered-point filter: a cell plus its neighbourhood must contain at
  // least this many points. Isolated / stray points never produce gain.
  int min_support_points;
  int neighbor_cell_radius;

  // Per-cycle budget, so a sudden burst of low-density points cannot dominate
  // the surface-point and frontier channels.
  int max_active_clusters;
  int max_active_points;
  int max_points_per_cluster;

  // Give-up policy: after max_failures unsuccessful attempts the cell is
  // abandoned until it genuinely disappears (with hysteresis).
  int max_failures;
  double cooldown_base_sec;
  double cooldown_max_sec;
  // Fallback for clusters that are never selected (no viewpoint can see them,
  // or they always lose to the surface/frontier channels): stop retrying after
  // they have been hanging around for this many cycles.
  int max_age_cycles;

  // Hysteresis: a cluster must be absent for this many cycles before its
  // history is forgotten. Prevents a one-frame dropout from resetting the
  // failed-attempt counter and re-opening an abandoned region.
  int missing_cycles_to_forget;

  LowDensityGuardParameters()
    : anchor_cell_size(3.0)
    , dedup_voxel_size(0.4)
    , min_support_points(5)
    , neighbor_cell_radius(1)
    , max_active_clusters(8)
    , max_active_points(200)
    , max_points_per_cluster(24)
    , max_failures(4)
    , cooldown_base_sec(8.0)
    , cooldown_max_sec(120.0)
    , max_age_cycles(120)
    , missing_cycles_to_forget(5)
  {
  }
};

struct LowDensityTarget
{
  Point3D position;
  // Gain multiplier in (0, 1]. Decays as attempts fail.
  double weight;
  // Stable identity. Pass this back through NoteServiced().
  uint64_t cluster_key;
  LowDensityTarget() : weight(0.0), cluster_key(0)
  {
  }
};

struct LowDensityGuardDiagnostics
{
  int raw_point_num;
  int dedup_point_num;
  int cell_num;
  int scattered_cell_num;
  int cooling_cell_num;
  int abandoned_cell_num;
  int active_cell_num;
  int output_point_num;
  LowDensityGuardDiagnostics()
    : raw_point_num(0)
    , dedup_point_num(0)
    , cell_num(0)
    , scattered_cell_num(0)
    , cooling_cell_num(0)
    , abandoned_cell_num(0)
    , active_cell_num(0)
    , output_point_num(0)
  {
  }
};

class LowDensityPointGuard
{
public:
  LowDensityPointGuard() : parameters_(), last_diagnostics_()
  {
  }
  explicit LowDensityPointGuard(const LowDensityGuardParameters& parameters)
    : parameters_(parameters), last_diagnostics_()
  {
  }

  void SetParameters(const LowDensityGuardParameters& parameters)
  {
    parameters_ = parameters;
  }
  void Reset()
  {
    cell_states_.clear();
  }
  const LowDensityGuardDiagnostics& GetDiagnostics() const
  {
    return last_diagnostics_;
  }

  bool IsSuppressed(uint64_t cluster_key) const
  {
    std::unordered_map<uint64_t, CellState>::const_iterator it = cell_states_.find(cluster_key);
    return it != cell_states_.end() && it->second.abandoned;
  }

  /**
   * @brief Called once per planning cycle with the freshly published cloud.
   * @param raw_points Low-density points published this cycle (may be empty).
   * @param now_sec Monotonic time in seconds, e.g. ros::Time::now().toSec().
   * @param targets Output: points allowed to contribute gain this cycle.
   */
  void Update(const std::vector<Point3D>& raw_points, double now_sec, std::vector<LowDensityTarget>& targets)
  {
    targets.clear();
    LowDensityGuardDiagnostics diag;
    diag.raw_point_num = static_cast<int>(raw_points.size());

    std::vector<Point3D> dedup_points;
    Deduplicate(raw_points, dedup_points);
    diag.dedup_point_num = static_cast<int>(dedup_points.size());

    std::unordered_map<uint64_t, CellBucket> buckets;
    BuildBuckets(dedup_points, buckets);
    diag.cell_num = static_cast<int>(buckets.size());
    ComputeSupport(buckets);

    std::unordered_set<uint64_t> seen_keys;
    for (std::unordered_map<uint64_t, CellBucket>::iterator it = buckets.begin(); it != buckets.end(); ++it)
    {
      const uint64_t key = it->first;
      CellBucket& bucket = it->second;
      seen_keys.insert(key);

      CellState& state = cell_states_[key];
      state.missing_cycles = 0;
      state.age_cycles++;

      // Surviving a cycle in which we already dispatched the robot for this
      // cluster is the definition of a failed attempt. The external publisher
      // is responsible for removing points that were successfully re-scanned.
      if (state.serviced_last_cycle)
      {
        state.failures++;
        const double exponent = static_cast<double>(std::max(0, state.failures - 1));
        const double cooldown =
            std::min(parameters_.cooldown_base_sec * std::pow(2.0, exponent), parameters_.cooldown_max_sec);
        state.cooldown_until = now_sec + cooldown;
        if (state.failures >= parameters_.max_failures)
        {
          state.abandoned = true;
        }
      }
      state.serviced_last_cycle = false;
      if (!state.abandoned && state.age_cycles > parameters_.max_age_cycles)
      {
        state.abandoned = true;
      }

      if (bucket.support < parameters_.min_support_points)
      {
        diag.scattered_cell_num++;
        continue;
      }
      if (state.abandoned)
      {
        diag.abandoned_cell_num++;
        continue;
      }
      if (now_sec < state.cooldown_until)
      {
        diag.cooling_cell_num++;
        continue;
      }

      bucket.weight = std::pow(0.5, static_cast<double>(state.failures));
      bucket.eligible = true;
      diag.active_cell_num++;
    }

    // Age out clusters that disappeared. Clear the serviced flag on a brief
    // dropout so a single missing frame is not misread as a failed attempt.
    for (std::unordered_map<uint64_t, CellState>::iterator it = cell_states_.begin(); it != cell_states_.end();)
    {
      if (seen_keys.find(it->first) == seen_keys.end())
      {
        it->second.missing_cycles++;
        it->second.serviced_last_cycle = false;
        if (it->second.missing_cycles > parameters_.missing_cycles_to_forget)
        {
          it = cell_states_.erase(it);
          continue;
        }
      }
      ++it;
    }

    std::vector<uint64_t> ordered_keys;
    for (std::unordered_map<uint64_t, CellBucket>::const_iterator it = buckets.begin(); it != buckets.end(); ++it)
    {
      if (it->second.eligible)
      {
        ordered_keys.push_back(it->first);
      }
    }
    std::sort(ordered_keys.begin(), ordered_keys.end(),
              [&buckets](uint64_t a, uint64_t b)
              {
                const CellBucket& lhs = buckets.find(a)->second;
                const CellBucket& rhs = buckets.find(b)->second;
                if (lhs.weight != rhs.weight)
                {
                  return lhs.weight > rhs.weight;
                }
                if (lhs.support != rhs.support)
                {
                  return lhs.support > rhs.support;
                }
                return a < b;
              });

    int point_budget = parameters_.max_active_points;
    int cluster_count = 0;
    for (std::size_t i = 0; i < ordered_keys.size(); i++)
    {
      if (cluster_count >= parameters_.max_active_clusters || point_budget <= 0)
      {
        break;
      }
      const CellBucket& bucket = buckets.find(ordered_keys[i])->second;
      const int point_num = std::min(static_cast<int>(bucket.points.size()),
                                     std::min(parameters_.max_points_per_cluster, point_budget));
      for (int j = 0; j < point_num; j++)
      {
        LowDensityTarget target;
        target.position = bucket.points[j];
        target.weight = bucket.weight;
        target.cluster_key = ordered_keys[i];
        targets.push_back(target);
      }
      point_budget -= point_num;
      cluster_count++;
    }
    diag.output_point_num = static_cast<int>(targets.size());
    last_diagnostics_ = diag;
  }

  /**
   * @brief Called after the viewpoint set is fixed, with the clusters whose
   * targets are covered by a selected viewpoint. These clusters spent path
   * budget this cycle; if they still exist next cycle they count as a failure.
   */
  void NoteServiced(const std::vector<uint64_t>& cluster_keys)
  {
    for (std::size_t i = 0; i < cluster_keys.size(); i++)
    {
      std::unordered_map<uint64_t, CellState>::iterator it = cell_states_.find(cluster_keys[i]);
      if (it != cell_states_.end() && !it->second.abandoned)
      {
        it->second.serviced_last_cycle = true;
      }
    }
  }

private:
  struct CellState
  {
    int failures;
    int age_cycles;
    int missing_cycles;
    double cooldown_until;
    bool abandoned;
    bool serviced_last_cycle;
    CellState()
      : failures(0)
      , age_cycles(0)
      , missing_cycles(0)
      , cooldown_until(-1.0)
      , abandoned(false)
      , serviced_last_cycle(false)
    {
    }
  };

  struct CellBucket
  {
    int ix;
    int iy;
    int iz;
    int support;
    double weight;
    bool eligible;
    std::vector<Point3D> points;
    CellBucket() : ix(0), iy(0), iz(0), support(0), weight(0.0), eligible(false)
    {
    }
  };

  static int FloorIndex(double value, double size)
  {
    return static_cast<int>(std::floor(value / size));
  }

  // Pack three signed grid indices into one collision-free 63-bit key.
  static uint64_t EncodeKey(int ix, int iy, int iz)
  {
    const int64_t kOffset = 1 << 20;
    const int64_t kSpan = 1 << 21;
    int64_t x = static_cast<int64_t>(ix) + kOffset;
    int64_t y = static_cast<int64_t>(iy) + kOffset;
    int64_t z = static_cast<int64_t>(iz) + kOffset;
    x = std::max<int64_t>(0, std::min<int64_t>(kSpan - 1, x));
    y = std::max<int64_t>(0, std::min<int64_t>(kSpan - 1, y));
    z = std::max<int64_t>(0, std::min<int64_t>(kSpan - 1, z));
    return static_cast<uint64_t>((x * kSpan + y) * kSpan + z);
  }

  void Deduplicate(const std::vector<Point3D>& raw_points, std::vector<Point3D>& dedup_points) const
  {
    const double s = parameters_.dedup_voxel_size > 0.0 ? parameters_.dedup_voxel_size : 0.01;
    std::unordered_set<uint64_t> occupied;
    dedup_points.clear();
    dedup_points.reserve(raw_points.size());
    for (std::size_t i = 0; i < raw_points.size(); i++)
    {
      const Point3D& p = raw_points[i];
      const uint64_t key = EncodeKey(FloorIndex(p.x, s), FloorIndex(p.y, s), FloorIndex(p.z, s));
      if (occupied.insert(key).second)
      {
        dedup_points.push_back(p);
      }
    }
  }

  void BuildBuckets(const std::vector<Point3D>& points, std::unordered_map<uint64_t, CellBucket>& buckets) const
  {
    const double s = parameters_.anchor_cell_size > 0.0 ? parameters_.anchor_cell_size : 1.0;
    buckets.clear();
    for (std::size_t i = 0; i < points.size(); i++)
    {
      const Point3D& p = points[i];
      const int ix = FloorIndex(p.x, s);
      const int iy = FloorIndex(p.y, s);
      const int iz = FloorIndex(p.z, s);
      const uint64_t key = EncodeKey(ix, iy, iz);
      CellBucket& bucket = buckets[key];
      if (bucket.points.empty())
      {
        bucket.ix = ix;
        bucket.iy = iy;
        bucket.iz = iz;
      }
      bucket.points.push_back(p);
    }
  }

  void ComputeSupport(std::unordered_map<uint64_t, CellBucket>& buckets) const
  {
    const int r = std::max(0, parameters_.neighbor_cell_radius);
    for (std::unordered_map<uint64_t, CellBucket>::iterator it = buckets.begin(); it != buckets.end(); ++it)
    {
      CellBucket& bucket = it->second;
      int support = 0;
      for (int dx = -r; dx <= r; dx++)
      {
        for (int dy = -r; dy <= r; dy++)
        {
          for (int dz = -r; dz <= r; dz++)
          {
            std::unordered_map<uint64_t, CellBucket>::const_iterator neighbor =
                buckets.find(EncodeKey(bucket.ix + dx, bucket.iy + dy, bucket.iz + dz));
            if (neighbor != buckets.end())
            {
              support += static_cast<int>(neighbor->second.points.size());
            }
          }
        }
      }
      bucket.support = support;
    }
  }

  LowDensityGuardParameters parameters_;
  LowDensityGuardDiagnostics last_diagnostics_;
  std::unordered_map<uint64_t, CellState> cell_states_;
};
}  // namespace low_density_point_ns
