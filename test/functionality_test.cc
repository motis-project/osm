#include <chrono>
#include <cmath>
#include <ranges>
#include <span>
#include <string_view>
#include <vector>

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

// --- 4. Check: point inside polygon (ray casting) ---
inline bool point_in_polygon(const Point& pt, const std::vector<Point>& ring) {
  auto inside = false;
  auto n = ring.size();
  const double eps = 1e-12;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const auto& pi = ring[i];
    const auto& pj = ring[j];
    // --- Check: point on edge ---
    double minx = std::min(pi.x, pj.x) - eps;
    double maxx = std::max(pi.x, pj.x) + eps;
    double miny = std::min(pi.y, pj.y) - eps;
    double maxy = std::max(pi.y, pj.y) + eps;
    double dx = pj.x - pi.x;
    double dy = pj.y - pi.y;
    if (std::abs(dx) < eps && std::abs(dy) < eps) continue;  // not real segment
    double cross = (pt.x - pi.x) * dy - (pt.y - pi.y) * dx;
    if (std::abs(cross) < eps && pt.x >= minx && pt.x <= maxx && pt.y >= miny &&
        pt.y <= maxy) {
      // happens in cases with "ways_in_multiple_rings",
      // when inner and outer share a way
      return true;  // point on edge = inside
    }
    // --- classic ray casting ---
    if (((pi.y > pt.y) != (pj.y > pt.y)) &&
        (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y + eps) + pi.x)) {
      inside = !inside;
    }
  }
  return inside;
}

TEST(relation_tests, functionality) {
  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/tmir/OSM/monaco-260324.osm.pbf",
                           cista::mmap::protection::READ}};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  std::atomic<size_t> ways_processed{0};
  size_t total_ways = 0;
  std::mutex rel_mutex;
  std::condition_variable rel_cv;
  bool ways_done = false;
  osm::PolygonManager mp_manager(false);

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
        mp_manager.save_ways_of_relation(id, members, tags);
      },
      pt);

  mp_manager.reserve_way_map(total_ways);
  r.reset_reader();

  int valid_count = 0;
  int skipped_count = 0;
  int not_built = 0;

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
        if (++ways_processed == total_ways) {
          std::lock_guard lock(rel_mutex);
          ways_done = true;
          rel_cv.notify_all();
        }
      },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        std::unique_lock lock(rel_mutex);
        rel_cv.wait(lock, [&] { return ways_done; });
        lock.unlock();
        assembler::polygon_area p_area =
            mp_manager.assemble_area(id, members, tags);

        // if (p_area.valid) {
        //   std::cout << "TEST START for id: " << id << std::endl;
        //   std::cout << "for: " << p_area.area.size() << std::endl;
        //   for (size_t i = 0; i < p_area.area.size(); ++i) {
        //     auto pair1 = p_area.area.at(i);
        //     pair1.reverse_complete_orientation();
        //     pair1.rotate_to_smallest();
        //     std::cout << "outer: [";
        //     for (osm::NodeRef nr : pair1.get_outer()) {
        //       std::cout << nr.ref_id << ", ";
        //     }
        //     std::cout << "] \n inners: " << pair1.offsets.size() - 1
        //               << std::endl;
        //     std::cout << "[";
        //     for (int k = 0; k < pair1.offsets.size(); ++k) {
        //       for (osm::NodeRef nrs : pair1.get_inner_at(k)) {
        //         std::cout << nrs.ref_id << ", ";
        //       }
        //     }
        //     std::cout << "]\n";
        //   }
        //   std::cout << std::endl;
        // }

        // =========== IS GEOMETRY RIGHT =========================
        for (size_t i = 0; i < p_area.area.size(); ++i) {
          auto& ap = p_area.area[i];
          ap.reverse_complete_orientation();
          ap.rotate_to_smallest();
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
          // check 4.
          for (size_t j = 0; j + 1 < ap.offsets.size(); ++j) {
            auto inner_pts = span_to_points(ap.get_inner_at(j));
            if (inner_pts.empty()) continue;
            EXPECT_TRUE(is_closed(inner_pts))
                << "Inner ring " << j << " is not closed, for id: " << id;
            EXPECT_TRUE(no_self_intersections(inner_pts))
                << "Inner ring " << j << " self-intersects, for id: " << id;
            EXPECT_GT(compute_area(inner_pts), 0.0)
                << "Inner ring " << j << " has zero area, for id: " << id;
            for (const auto& p : inner_pts) {
              EXPECT_TRUE(point_in_polygon(p, outer_pts))
                  << "Inner ring " << j << " is outside outer ring " << i
                  << ", for id: " << id;
            }
          }
        }
        // ========================================================

        bool is_multipolygon = mp_manager.is_area(tags);
        if (is_multipolygon) {
          if (!p_area.valid && !p_area.missing_flag) {
            not_built++;
          }
          if (p_area.valid) {
            valid_count++;
            EXPECT_GT(p_area.get_all_outers().size(), 0);
            auto inners = 0;
            for (size_t i = 0; i < p_area.area.size(); ++i) {
              auto outer = p_area.area[i].get_outer();
              EXPECT_FALSE(outer.empty()) << "Outer " << i << " is empty";
              inners += p_area.area[i].get_inners().size();
            }
          }
        } else {
          EXPECT_FALSE(p_area.valid);
          skipped_count++;
        }
        // -----------------------------------------------------------------
      },
      pt);

  mp_manager.all_stats.print_stats();

  std::cout << std::endl;
  std::cout << "not built: " << not_built << std::endl;

  EXPECT_GT(valid_count, 0);
  EXPECT_GT(skipped_count, 0);
}