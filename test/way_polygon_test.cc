#include <chrono>
#include <ranges>
#include <string_view>

#include "fmt/ranges.h"
#include "gtest/gtest.h"
#include "utl/progress_tracker.h"

#include "osm/assembler/assembler_types.h"
#include "osm/decoder.h"
#include "osm/filing/tmp_file.h"
#include "osm/hnidx/hybrid_node_index.h"
#include "osm/inflate.h"
#include "osm/memory.h"
#include "osm/mp_manager.h"
#include "osm/osm.h"
#include "osm/parallel.h"

// --- makes helper functions more readable ---
struct Point {
  std::int64_t x, y;
};

// convert std::span<const NodeRef> into std::vector<Point>
inline std::vector<Point> span_to_points(std::span<const osm::NodeRef> span) {
  std::vector<Point> pts;
  pts.reserve(span.size());
  for (const auto& n : span) {
    pts.push_back({n.location().x(), n.location().y()});
  }
  return pts;
}

// --- 1. Check: is ring closed ---
inline bool is_closed(const std::vector<Point>& ring) {
  if (ring.size() < 4) return false;
  return std::abs(ring.front().x - ring.back().x) < 1e-9 &&
         std::abs(ring.front().y - ring.back().y) < 1e-9;
}

// --- 2. Check: no self intersections ---
inline bool segments_intersect(const Point& a,
                               const Point& b,
                               const Point& c,
                               const Point& d) {
  auto cross = [](const Point& p0, const Point& p1, const Point& p2) {
    return (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  };
  auto ccw = [&](const Point& p0, const Point& p1, const Point& p2) {
    return cross(p0, p1, p2) > 0;
  };
  return (ccw(a, c, d) != ccw(b, c, d)) && (ccw(a, b, c) != ccw(a, b, d));
}
inline bool no_self_intersections(const std::vector<Point>& ring) {
  auto n = ring.size();
  if (n < 4) return true;  // trivial
  for (auto i = 0; i < n - 1; ++i) {
    for (auto j = i + 1; j < n - 1; ++j) {
      // ignore neighbouring segments
      if (std::abs((int)i - (int)j) <= 1) continue;
      if (i == 0 && j == n - 2) continue;  // first and last segment point
      if (segments_intersect(ring[i], ring[i + 1], ring[j], ring[j + 1])) {
        return false;
      }
    }
  }
  return true;
}

// --- 3. Check: area > 0 (Shoelace formula) ---
inline double compute_area(const std::vector<Point>& ring) {
  auto sum = 0.0;
  auto n = ring.size();
  for (auto i = 0; i < n - 1; ++i) {
    sum += ring[i].x * ring[i + 1].y - ring[i + 1].x * ring[i].y;
  }
  return std::abs(sum) / 2.0;
}

TEST(way_tests, way_areas_monaco) {
  auto r = osm::raw_reader{"/home/tmir/OSM/berlin-251113.osm.pbf"};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  std::atomic_uint64_t relations_count = 0;
  std::atomic_uint64_t worked_count = 0;
  std::atomic_uint64_t total_ways = 0;
  osm::PolygonManager mp_manager(true);

  auto tmp_dname = std::filesystem::temp_directory_path();
  auto const node_idx_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "idx.bin").generic_string()};
  auto const node_dat_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "dat.bin").generic_string()};
  tiles::hybrid_node_idx node_idx{node_idx_file.fileno(),
                                  node_dat_file.fileno()};
  tiles::hybrid_node_idx_builder node_idx_builder{node_idx};

  //   PASS 1: nodes & ways
  osm::decode_primitive_parallel(
      r, node_idx_builder, true, true, true,
      [&](std::int64_t const id, geo::latlng const& pos, auto&& tags) {},
      [&](std::int64_t, auto&&, auto&&) { total_ways++; },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        relations_count++;
        mp_manager.save_ways_of_relation(id, members, tags);
      },
      pt);

  mp_manager.reserve_way_map(total_ways);
  r.reset_reader();

  // PASS 2: areas
  osm::decode_primitive_parallel(
      r, node_idx_builder, false, true, true,
      [&](std::int64_t, geo::latlng const&, auto&&) {},
      [&](std::int64_t const id, auto&& refs, auto&& tags) {
        std::vector<osm::NodeRef> way_node_refs;
        std::int64_t acc = 0;
        for (auto r : refs) {
          acc += r;
          way_node_refs.emplace_back(acc);
        }
        osm::Way tempway{id, way_node_refs};
        tiles::update_locations_of_way(node_idx, tempway);
        auto area_result = mp_manager.save_ways(tempway, tags);
        if (area_result.has_value()) {
          worked_count++;
          const auto& poly = area_result.value();
          // =========== IS GEOMETRY RIGHT ====================================
          for (size_t i = 0; i < poly.area.size(); ++i) {
            const auto& ap = poly.area[i];
            auto outer_pts = span_to_points(ap.get_outer());
            // check 1.
            EXPECT_TRUE(is_closed(outer_pts))
                << "Outer ring " << i << " is not closed, for id: " << id;
            // check 2.
            EXPECT_TRUE(no_self_intersections(outer_pts))
                << "Outer ring " << i << " self-intersects, for id: " << id;
            // check 3.
            EXPECT_GT(compute_area(outer_pts), 0.0)
                << "Outer ring " << i << " has zero area, for id: " << id;
          }
          // ==================================================================
        }
      },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        // do nothing -> this is the way area test
      },
      pt);

  std::cout << " \t Results: " << std::endl;
  std::cout << " \t number of read ways: " << total_ways << std::endl;
  std::cout << " \t number of build areas: " << worked_count << std::endl;
}