#pragma once

#include <cstdint>

#include "osm/types.h"

namespace osm {

template <typename WayFn>
struct way_handler {
  void operator()(auto& local,
                  std::int64_t const id,
                  auto&& refs,
                  auto&& tags) const {
    auto w = way{};
    w.id = id;
    for (auto const ref : refs) {
      w.node_refs.emplace_back(node_ref{ref, {}});
    }
    on_way_(local, std::move(w), tags);
  }

  WayFn on_way_;
};

template <typename WayFn>
way_handler(WayFn&&) -> way_handler<std::decay_t<WayFn>>;

}  // namespace osm
