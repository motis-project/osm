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

TEST(timing, test) {
  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/tmir/OSM/berlin-251113.osm.pbf",
                           cista::mmap::protection::READ}};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  std::atomic_uint64_t relations_count = 0;
  std::atomic_uint64_t worked_count = 0;
  std::atomic_uint64_t total_ways = 0;
  osm::PolygonManager mp_manager(false);
  std::atomic<size_t> ways_processed{0};
  bool ways_done = false;
  std::mutex rel_mutex;
  std::condition_variable rel_cv;

  auto tmp_dname = std::filesystem::temp_directory_path();
  auto const node_idx_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "idx.bin").generic_string()};
  auto const node_dat_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "dat.bin").generic_string()};
  tiles::hybrid_node_idx node_idx{node_idx_file.fileno(),
                                  node_dat_file.fileno()};
  tiles::hybrid_node_idx_builder node_idx_builder{node_idx};

  auto start1 = std::chrono::high_resolution_clock::now();
  //   PASS 1: nodes & ways
  osm::decode_primitive_parallel(
      r, node_idx_builder, true, true, true,
      [&](std::int64_t const id, geo::latlng const& pos, auto&& tags) {},
      [&](std::int64_t, auto&&, auto&&) { total_ways++; },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        mp_manager.save_ways_of_relation(id, members, tags);
      },
      pt);
  auto end1 = std::chrono::high_resolution_clock::now();
  auto duration_pass1 =
      std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);

  mp_manager.reserve_way_map(total_ways);
  r.reset_reader();

  auto start2 = std::chrono::high_resolution_clock::now();
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
      },
      pt);
  auto end2 = std::chrono::high_resolution_clock::now();
  auto duration_pass2 =
      std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);

  std::cout << "\t Duration Pass1: " << duration_pass1.count()
            << "\t Duration Pass2: " << duration_pass2.count() << std::endl;
}