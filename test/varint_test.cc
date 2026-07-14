#include <array>
#include <cstdint>
#include <ranges>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "osm/ranges_chunk.h"
#include "osm/varint.h"

TEST(osm, varint) {
  auto buf = std::array<char, protozero::max_varint_length * 10U>{};
  auto const n1 =
      protozero::write_varint(buf.data(), protozero::encode_zigzag64(123));
  auto const n2 =
      protozero::write_varint(buf.data() + n1, protozero::encode_zigzag64(456));
  auto const n3 = protozero::write_varint(buf.data() + n1 + n2,
                                          protozero::encode_zigzag64(789));
  protozero::write_varint(buf.data() + n1 + n2 + n3, 0);

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

TEST(osm, varint_forward_iterator) {
  static_assert(std::forward_iterator<osm::varint<std::uint32_t>::iterator>);
  static_assert(std::ranges::forward_range<osm::varint<std::uint32_t>>);

  auto const v =
      osm::varint<std::uint32_t>{std::string_view{"\x01\x02\x03\x04\x05\x06", 6}};

  auto a = v.begin();
  auto b = a;
  ++b;
  ++b;
  EXPECT_EQ(1, *a);
  EXPECT_EQ(3, *b);
  EXPECT_NE(a, b);
  EXPECT_EQ(a, v.begin());
  EXPECT_EQ(*a, *v.begin());

  auto it = v.begin();
  EXPECT_EQ(1, *it);
  EXPECT_EQ(1, *it);

  auto const collect = [&]() {
    auto out = std::vector<std::int64_t>{};
    for (auto const x : v) {
      out.push_back(x);
    }
    return out;
  };
  auto const expected = std::vector<std::int64_t>{1, 2, 3, 4, 5, 6};
  EXPECT_EQ(expected, collect());
  EXPECT_EQ(expected, collect());
}

TEST(osm, varint_chunk) {
  auto const v =
      osm::varint<std::uint32_t>{std::string_view{"\x01\x02\x03\x04\x05\x06", 6}};

  auto const pairs =
      v | std::views::chunk(2) | std::views::transform([](auto&& y) {
        auto it = std::ranges::begin(y);
        auto const k = *it;
        auto const val = *++it;
        return std::pair{k, val};
      });

  auto got = std::vector<std::pair<std::int64_t, std::int64_t>>{};
  for (auto const [k, val] : pairs) {
    got.emplace_back(k, val);
  }

  EXPECT_EQ((std::vector<std::pair<std::int64_t, std::int64_t>>{
                {1, 2}, {3, 4}, {5, 6}}),
            got);
}

TEST(osm, varint_chunk_partial_tail) {
  auto const chunks_of = [](std::string_view const bytes, int const n) {
    auto const v = osm::varint<std::uint32_t>{bytes};
    auto out = std::vector<std::vector<std::int64_t>>{};
    for (auto&& c : v | std::views::chunk(n)) {
      auto& chunk = out.emplace_back();
      for (auto const x : c) {
        chunk.push_back(x);
      }
    }
    return out;
  };

  EXPECT_EQ((std::vector<std::vector<std::int64_t>>{{1, 2}, {3, 4}, {5}}),
            chunks_of(std::string_view{"\x01\x02\x03\x04\x05", 5}, 2));
}
