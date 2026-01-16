#pragma once

#include "osm/decoder.h"

namespace osm {

struct multi_polygons {
  std::vector<std::int64_t> ways_;
  std::vector<std::vector<std::int64_t>> way_nodes_;

  std::vector<polygon> multipolygon_;
};

struct area {
  void reset() {}
};

inline bool is_area(auto&& tags) {
  return true;
}

void save_ways(multi_polygons& mp, std::int64_t const id, auto&& members, auto&& tags) {
  if (!is_area(tags)) {
    return;
  }
  for (auto const [ref, role, type] : members) {
    if (type != member_type::kWay) {
      continue;
    }
    mp.ways_.emplace_back(ref);
  }
}

void assemble_area(multi_polygons& mp, std::int64_t const id, auto&& members, auto&& tags, area& a) {
  if (!is_area(tags)) {
    return;
  }

  a.reset();
  // ... build area (assembler code from libosmium)
}

}  //