#include "osm/osm.h"

#include "fmt/ranges.h"

#include "gtest/gtest.h"

#include "utl/progress_tracker.h"

#include "osm/decoder.h"
#include "osm/inflate.h"
#include "osm/memory.h"

#include "osm/parallel.h"

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
  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/felix/Downloads/germany-latest.osm.pbf",
                           cista::mmap::protection::READ}};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  auto const n_threads = std::thread::hardware_concurrency();

  auto print_mtx = std::mutex{};

  auto first_way_buffer_start_offset = std::atomic_uint64_t{};

  // PASS 1
  auto mp = osm::multi_polygons{};
  osm::decode_primitive_parallel(
      r, true, false, true,
      [&](std::int64_t const id, geo::latlng const& pos, auto&& tags) {
        // TODO update hybrid node builder
      },
      [&](std::int64_t, auto&&, auto&&) {},
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        osm::save_ways(mp, id, members, tags);
      },
      pt);

  // PASS 2
  osm::decode_primitive_parallel(
      r, false, true, true, [&](std::int64_t, geo::latlng const&, auto&&) {},
      [&](std::int64_t const id, auto&& refs, auto&& tags) {
        // TODO save nodes of ways
      },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        auto a = osm::assemble_area(mp, id, members, tags);
        // TODO do something with final area
        // a.outer_rings()
      },
      pt);
}