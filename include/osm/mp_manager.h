#pragma once

#include <algorithm>
#include <mutex>
#include <optional>

#include "osm/assembler.h"
#include "osm/assembler/assemble_rings.h"
#include "osm/assembler/assembler_stats.h"
#include "osm/assembler/assembler_types.h"
#include "osm/decoder.h"

namespace osm {

struct multi_polygon {
  std::int64_t relation_id;
  std::vector<std::int64_t> ways_refs;
};

struct PolygonManager {
  std::mutex stats_mtx;
  assembler::area_stats all_stats{};
  bool assemble_way_polygons_ = false;
  std::mutex mp_vec_mtx;
  std::vector<osm::multi_polygon> mp_vec_ = std::vector<osm::multi_polygon>{};
  std::mutex ways_vec_mtx;
  std::unordered_map<object_id_type, osm::Way> all_ways_;
  PolygonManager(bool assemble_way_polygons = false)
      : assemble_way_polygons_(assemble_way_polygons) {}

  template <typename Tags>
  inline bool is_area(Tags&& tags) {
    for (auto const& [key, value] : tags) {
      if (key == "type" && (value == "multipolygon" || value == "boundary")) {
        return true;
      }
    }
    return false;
  }

  template <typename Tags>
  inline bool is_way_area(Tags&& tags) {
    bool has_line_indicator = false;
    bool has_area_indicator = false;
    for (auto const& [key, value] : tags) {
      if (key == "area") {
        if (value == "yes") {
          return true;
        } else if (value == "no") {
          return false;
        }
        continue;
      }
      // area indicator (implicit)
      if (key == "building" || key == "landuse" || key == "natural" ||
          key == "amenity" || key == "leisure" || key == "tourism" ||
          (key == "waterway" && value == "riverbank") ||
          (key == "waterway" && value == "dock") ||
          (key == "power" && (value == "plant" || value == "substation"))) {
        has_area_indicator = true;
        continue;
      }
      // line indicator overrides area indicator
      if (key == "highway" || key == "railway" ||
          (key == "waterway" && value != "riverbank" && value != "dock") ||
          (key == "barrier" || (key == "man_made" && value == "pier"))) {
        has_line_indicator = true;
      }
    }
    return has_area_indicator && !has_line_indicator;
  }

  int count_non_areas = 0;
  template <typename Members, typename Tags>
  void save_ways_of_relation(std::int64_t const id,
                             Members&& members,
                             Tags&& tags) {
    if (!is_area(tags)) {
      count_non_areas++;
      return;
    }
    auto mp = osm::multi_polygon{};
    mp.relation_id = id;
    for (auto const [ref, role, type] : members) {
      if (type != osm::member_type::kWay) {
        continue;
      }
      mp.ways_refs.emplace_back(ref);
    }
    std::lock_guard<std::mutex> lock(mp_vec_mtx);
    mp_vec_.emplace_back(std::move(mp));
  }

  void reserve_way_map(size_t expected_count) {
    all_ways_.reserve(expected_count);
  }

  template <typename Tags>
  std::optional<assembler::polygon_area> save_ways(osm::Way way, Tags&& tags) {
    {
      std::lock_guard<std::mutex> lock(ways_vec_mtx);
      all_ways_.insert({way.id, way});
    }
    // for the more defensive version, include is_way_area check.
    if (assemble_way_polygons_) {  // && is_way_area(tags)) {
      assembler::polygon_area a(way.id);
      a.from_way = true;
      assembler::assembly assemble = assembler::assembly{};
      if (assemble.assembling_area_from_way(way, a)) {
        return a;
      }
    }
    return std::nullopt;
  }

  std::vector<const osm::Way*> make_const_way_ptrs(
      const std::vector<object_id_type>& ids) {
    std::vector<const osm::Way*> ptrs = {};
    ptrs.reserve(ids.size());
    for (const auto& id : ids) {
      auto it = all_ways_.find(id);
      if (it != all_ways_.end()) {
        ptrs.push_back(&(*it).second);
      }
    }
    std::sort(
        ptrs.begin(), ptrs.end(),
        [](const osm::Way* a, const osm::Way* b) { return a->id < b->id; });
    return ptrs;
  }

  template <typename Members, typename Tags>
  assembler::polygon_area assemble_area(std::int64_t const id,
                                        Members&& members,
                                        Tags&& tags) {
    assembler::polygon_area a(id);
    if (!is_area(tags)) {
      return a;
    }
    assembler::assembly assemble = assembler::assembly{};
    bool worked = false;
    osm::Relation r = {id, members};
    std::vector<const osm::Way*> ways = {};
    for (auto elem : mp_vec_) {
      if (elem.relation_id == id) {
        ways = make_const_way_ptrs(elem.ways_refs);
        break;
      }
    }
    worked = assemble.assembling_area_from_relation(r, ways, a);
    {
      // accumulate all statistics
      std::lock_guard<std::mutex> lock(stats_mtx);
      all_stats += a.pa_stats;
    }
    return a;
  }
};  // struct PolygonManager
}  // namespace osm