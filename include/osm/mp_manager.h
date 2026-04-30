#pragma once

#include <algorithm>
#include <mutex>
#include <optional>
#include <string_view>

#include "cista/hash.h"

#include "utl/helpers/algorithm.h"

#include "osm/assembler.h"
#include "osm/assembler/assembler_stats.h"
#include "osm/assembler/assembler_types.h"
#include "osm/decoder.h"

namespace osm {

struct multi_polygon {
  std::int64_t relation_id;
  std::vector<std::int64_t> ways_refs;
};

template <typename Tags>
inline bool is_area(Tags&& tags) {
  for (auto const& [key, value] : tags) {
    switch (cista::hash(std::string_view{key})) {
      using namespace std::string_view_literals;
      case cista::hash("type"):
        if (value == "multipolygon"sv || value == "boundary"sv) {
          return true;
        }
        break;
    }
  }
  return false;
}

template <typename Tags>
inline bool is_way_area(Tags&& tags) {
  auto has_line_indicator = false;
  auto has_area_indicator = false;
  for (auto const& [key, value] : tags) {
    switch (cista::hash(std::string_view{key})) {
      using namespace std::string_view_literals;
      case cista::hash("area"):
        if (value == "yes"sv) {
          return true;
        } else if (value == "no"sv) {
          return false;
        }
        break;
      case cista::hash("building"):
      case cista::hash("landuse"):
      case cista::hash("natural"):
      case cista::hash("amenity"):
      case cista::hash("leisure"):
      case cista::hash("tourism"): has_area_indicator = true; break;
      case cista::hash("waterway"):
        if (value == "riverbank"sv || value == "dock"sv) {
          has_area_indicator = true;
        } else {
          has_line_indicator = true;
        }
        break;
      case cista::hash("power"):
        if (value == "plant"sv || value == "substation"sv) {
          has_area_indicator = true;
        }
        break;
      case cista::hash("highway"):
      case cista::hash("railway"):
      case cista::hash("barrier"): has_line_indicator = true; break;
      case cista::hash("man_made"):
        if (value == "pier"sv) {
          has_line_indicator = true;
        }
        break;
    }
  }
  return has_area_indicator && !has_line_indicator;
}

struct polygon_manager {
  explicit polygon_manager(bool assemble_way_polygons = false)
      : assemble_way_polygons_{assemble_way_polygons} {}

  template <typename Members, typename Tags>
  void save_ways_of_relation(std::int64_t const id,
                             Members&& members,
                             Tags&& tags) {
    if (!is_area(tags)) {
      ++count_non_areas;
      return;
    }

    auto mp = multi_polygon{};
    mp.relation_id = id;
    for (auto const [ref, role, type] : members) {
      if (type == member_type::kWay) {
        mp.ways_refs.emplace_back(ref);
      }
    }

    std::lock_guard<std::mutex> lock(mp_vec_mtx);
    mp_vec_.emplace_back(std::move(mp));
  }

  void reserve_way_map(std::size_t expected_count) {
    all_ways_.reserve(expected_count);
  }

  template <typename Tags>
  std::optional<polygon_area> save_ways(way way, Tags&& tags) {
    auto result = std::optional<polygon_area>{};
    // for the more defensive version, include is_way_area check.
    if (assemble_way_polygons_) {  // && is_way_area(tags)) {
      auto a = polygon_area{way.id};
      a.from_way = true;
      auto assemble = assembly{};
      if (assemble.assembling_area_from_way(way, a)) {
        result = a;
      }
    }
    {
      auto lock = std::lock_guard{ways_vec_mtx};
      all_ways_.insert({way.id, std::move(way)});
    }
    return result;
  }

  std::vector<way const*> make_const_way_ptrs(
      std::vector<object_id_type> const& ids) {
    auto ptrs = std::vector<way const*>{};
    ptrs.reserve(ids.size());
    for (auto const& id : ids) {
      auto it = all_ways_.find(id);
      if (it != all_ways_.end()) {
        ptrs.push_back(&(*it).second);
      }
    }
    utl::sort(ptrs, [](way const* a, way const* b) { return a->id < b->id; });
    return ptrs;
  }

  template <typename Members, typename Tags>
  polygon_area assemble_area(std::int64_t const id,
                             Members&& members,
                             Tags&& tags) {
    auto a = polygon_area{id};
    if (!is_area(tags)) {
      return a;
    }

    auto assemble = assembly{};
    auto worked = false;
    auto r = relation{id, members};
    auto ways = std::vector<const way*>{};
    for (auto elem : mp_vec_) {
      if (elem.relation_id == id) {
        ways = make_const_way_ptrs(elem.ways_refs);
        break;
      }
    }

    worked = assemble.assembling_area_from_relation(r, ways, a);

    all_stats += a.pa_stats;

    return a;
  }

  area_stats all_stats{};
  bool assemble_way_polygons_{false};
  std::mutex mp_vec_mtx;
  std::vector<multi_polygon> mp_vec_{};
  std::mutex ways_vec_mtx;
  std::unordered_map<object_id_type, way> all_ways_;
  std::atomic_uint64_t count_non_areas{0};
};

}  // namespace osm