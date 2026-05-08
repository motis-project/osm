#include "osm/raw_reader.h"

#include <atomic>
#include <filesystem>
#include <variant>

#include "boost/fiber/all.hpp"

#include "fmt/ranges.h"

#include "gtest/gtest.h"

#include "utl/progress_tracker.h"

#include "osm/memory.h"
#include "osm/mp_manager.h"
#include "osm/node_idx.h"
#include "osm/parse.h"
#include "osm/way_handler.h"

namespace bf = boost::fibers;

TEST(osm, varint) {
  auto buf = std::array<char, protozero::max_varint_length * 10U>{};
  auto const n1 =
      protozero::write_varint(buf.data(), protozero::encode_zigzag64(123));
  auto const n2 =
      protozero::write_varint(buf.data() + n1, protozero::encode_zigzag64(456));
  auto const n3 = protozero::write_varint(buf.data() + n1 + n2,
                                          protozero::encode_zigzag64(789));
  auto const n4 = protozero::write_varint(buf.data() + n1 + n2 + n3, 0);

  EXPECT_EQ(0, buf[n1 + n2 + n3]);

  auto const size = static_cast<std::size_t>(n1 + n2 + n3);
  auto x = static_cast<char const*>(buf.data());
  EXPECT_EQ(123,
            protozero::decode_zigzag64(protozero::decode_varint(&x, x + size)));

  auto const empty = osm::varint<std::int64_t>{std::string_view{}};
  EXPECT_EQ(empty.begin(), empty.end());

  auto const v = osm::varint<std::int64_t>{
      std::string_view{buf.data(), static_cast<std::size_t>(n1 + n2 + n3)}};
  auto it = v.begin();
  EXPECT_NE(it, v.end());
  EXPECT_EQ(123, *it);

  ++it;
  EXPECT_NE(it, v.end());
  EXPECT_EQ(456, *it);

  ++it;
  EXPECT_NE(it, v.end());
  EXPECT_EQ(789, *it);

  ++it;
  EXPECT_EQ(it, v.end());
}

TEST(a, b) {
  auto bars = utl::global_progress_bars{false};

  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/felix/Downloads/germany-latest.osm.pbf",
                           cista::mmap::protection::READ}};

  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size() * 2U);

  auto const idx_path =
      std::filesystem::temp_directory_path() / "osm_node_idx.bin";
  auto node_idx = osm::node_idx_t{
      cista::mmap{idx_path.generic_string().c_str()}};
  auto node_handler = osm::node_idx_handler{node_idx};

  auto mp_manager = osm::polygon_manager{true};

  auto n_ways = std::atomic_uint64_t{0U};
  auto n_rels = std::atomic_uint64_t{0U};
  auto n_areas = std::atomic_uint64_t{0U};

  // PASS 1: nodes -> idx; remember way-refs for each relation; count ways.
  osm::parse_osm(
      r,
      [] { return std::monostate{}; },
      node_handler,
      [&](auto&, std::int64_t const, auto&&, auto&&) { ++n_ways; },
      [&](auto&, std::int64_t const id, auto&& members, auto&& tags) {
        ++n_rels;
        mp_manager.save_ways_of_relation(id, members, tags);
      },
      pt->update_fn());

  mp_manager.reserve_way_map(static_cast<std::size_t>(n_ways.load()));
  r.reset();

  // PASS 2: rebuild ways with locations -> mp_manager; assemble areas from
  // relations once all ways are in. Synchronization uses bf primitives so
  // waiting fibers don't stall the work-stealing scheduler.
  auto ways_processed = std::atomic_uint64_t{0U};
  auto ways_done = false;
  auto rel_mtx = bf::mutex{};
  auto rel_cv = bf::condition_variable{};
  auto const total_ways = n_ways.load();

  osm::parse_osm(
      r,
      [] { return std::monostate{}; },
      [](auto&, std::int64_t const, geo::latlng const&, auto&&) {},
      osm::way_handler{[&](auto&, osm::way&& w, auto&& tags) {
        osm::update_locations_of_way(node_idx, w);
        if (auto a = mp_manager.save_ways(std::move(w), tags);
            a && a->valid) {
          ++n_areas;
        }
        if (++ways_processed == total_ways) {
          auto lock = std::lock_guard{rel_mtx};
          ways_done = true;
          rel_cv.notify_all();
        }
      }},
      [&](auto&, std::int64_t const id, auto&& members, auto&& tags) {
        auto lock = std::unique_lock{rel_mtx};
        rel_cv.wait(lock, [&] { return ways_done; });
        lock.unlock();
        auto area = mp_manager.assemble_area(id, members, tags);
        if (area.valid) {
          ++n_areas;
        }
      },
      pt->update_fn());

  std::cout << "ways: " << n_ways << "\n"
            << "relations: " << n_rels << "\n"
            << "valid areas: " << n_areas << "\n";

  const osm::memory_usage memory;
  std::cout << "\nMemory used: " << memory.peak() << " MBytes\n";
}
