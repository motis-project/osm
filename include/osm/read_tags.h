#pragma once

#include <array>
#include <string_view>
#include <tuple>
#include <utility>

#include "cista/hash.h"

namespace osm {

// Single linear pass over `tags`, returning the value of each requested key
// (empty if the key is absent). `tags` may be a range of osmium-style tags
// exposing `.key()`/`.value()`, or of `[key, value]` pairs/tuples as produced
// by `decode_primitive`. Replaces a separate per-key lookup scan.
//
// Marked always-inline so the requested-key hashes fold to constants at the
// call site; per tag it is then one hash plus a few integer compares -- the
// same work a hand-written `switch` does (a switch over 64-bit cista hashes
// lowers to a comparison tree, not a jump table).
template <typename Tags, typename... Keys>
[[gnu::always_inline]] inline std::array<std::string_view, sizeof...(Keys)>
read_tags(Tags const& tags, Keys const&... keys) {
  auto const want = std::array{cista::hash(std::string_view{keys})...};
  auto found = std::array<std::string_view, sizeof...(Keys)>{};
  for (auto const& t : tags) {
    auto const key = [&]() -> std::string_view {
      if constexpr (requires { t.key(); }) {
        return t.key();
      } else {
        return std::get<0>(t);
      }
    }();
    auto const h = cista::hash(key);
    for (auto i = 0U; i != want.size(); ++i) {
      if (h == want[i]) {
        if constexpr (requires { t.value(); }) {
          found[i] = t.value();
        } else {
          found[i] = std::get<1>(t);
        }
        break;
      }
    }
  }
  return found;
}

}  // namespace osm
