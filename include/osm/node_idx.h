#pragma once

#include <cmath>
#include <cstdint>

#include "cista/containers/mmap_vec.h"

#include "geo/fixed_latlng.h"
#include "geo/latlng.h"

#include "osm/types.h"

namespace osm {

using node_idx_t = cista::basic_mmap_vec<geo::fixed_latlng, std::uint64_t>;

constexpr auto kDefaultNodeIndexCapacity = std::uint64_t{1} << 36U;

struct node_idx_handler {
  explicit node_idx_handler(node_idx_t& idx,
                            std::uint64_t capacity = kDefaultNodeIndexCapacity)
      : idx_{idx} {
    if (idx_.size() < capacity) {
      idx_.reserve(capacity);
      idx_.used_size_ = capacity;
    }
  }

  void operator()(auto& /*local*/,
                  std::int64_t const id,
                  geo::latlng const& pos,
                  auto&& /*tags*/) const {
    auto const abs_id = static_cast<std::uint64_t>(std::abs(id));
    idx_[abs_id] = geo::fixed_latlng::from_latlng(pos);
  }

  node_idx_t& idx_;
};

// Fill in the node_ref locations of a way from the node index, indexed by
// each ref's absolute OSM id.
inline void update_locations_of_way(node_idx_t const& idx, way& w) {
  for (auto& nr : w.nodes()) {
    auto const abs_id = static_cast<std::uint64_t>(std::abs(nr.ref()));
    auto const& fll = idx[abs_id];
    nr.set_location(location{fll.lat_, fll.lng_});
  }
}

}  // namespace osm
