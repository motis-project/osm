#pragma once

#include <algorithm>
#include <iostream>
#include <mutex>
#include <optional>
#include <string_view>

#include "cista/hash.h"

#include "utl/erase_duplicates.h"
#include "utl/helpers/algorithm.h"

#include "osm/assembler.h"
#include "osm/assembler/assembler_stats.h"
#include "osm/assembler/assembler_types.h"
#include "osm/decoder.h"

namespace osm {

struct multi_polygon {
  std::int64_t relation_id_;
  std::vector<std::int64_t> ways_refs_;
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
  // debug: keep node ids of relation member ways and enable the assembler's
  // problem reports and debug output (to std::cerr).
  explicit polygon_manager(bool assemble_way_polygons = false,
                           bool debug = false)
      : assemble_way_polygons_{assemble_way_polygons}, debug_{debug} {}

  template <typename Members, typename Tags>
  void save_ways_of_relation(std::int64_t const id,
                             Members&& members,
                             Tags&& tags) {
    if (!is_area(tags)) {
      ++count_non_areas_;
      return;
    }

    auto mp = multi_polygon{};
    mp.relation_id_ = id;
    for (auto const [ref, role, type] : members) {
      if (type == member_type::kWay) {
        mp.ways_refs_.emplace_back(ref);
      }
    }

    std::lock_guard<std::mutex> lock(mp_vec_mtx_);
    mp_index_.emplace(id, mp_vec_.size());
    mp_vec_.emplace_back(std::move(mp));
  }

  void reserve_way_map(std::size_t expected_count) {
    way_to_id_.reserve(expected_count);
    osm_id_to_way_.reserve(expected_count);
  }

  void index_relation_members() {
    member_way_ids_.clear();
    for (auto const& mp : mp_vec_) {
      member_way_ids_.insert(end(member_way_ids_), begin(mp.ways_refs_),
                             end(mp.ways_refs_));
    }
    utl::erase_duplicates(member_way_ids_);
    members_indexed_ = true;
    way_to_id_.reserve(member_way_ids_.size());
    osm_id_to_way_.reserve(member_way_ids_.size());
  }

  bool is_relation_member(object_id_type const id) const {
    return std::binary_search(begin(member_way_ids_), end(member_way_ids_), id);
  }

  template <typename Tags>
  std::optional<polygon_area> save_ways(way way, Tags&& tags) {
    auto result = std::optional<polygon_area>{};
    // for the more defensive version, include is_way_area check.
    if (assemble_way_polygons_) {  // && is_way_area(tags)) {
      auto a = polygon_area{way.id};
      a.from_way = true;
      auto assemble = make_assembly();
      if (assemble.assembling_area_from_way(way, a)) {
        result = a;
      }
    }

    // Only ways referenced by an area are used assemble_area. The assembler
    // compares locations only, so node ids are stored in debug mode only.
    if (!members_indexed_ || is_relation_member(way.id)) {
      auto lock = std::lock_guard{ways_vec_mtx_};
      auto const idx =
          way_idx_t{static_cast<cista::base_t<way_idx_t>>(way_to_id_.size())};
      way_to_id_.emplace_back(way.id);
      auto locations = way_locations_.add_back_sized(way.node_refs.size());
      for (auto i = 0U; i != way.node_refs.size(); ++i) {
        locations[i] = way.node_refs[i].location();
      }
      if (debug_) {
        auto ids = way_node_ids_.add_back_sized(way.node_refs.size());
        for (auto i = 0U; i != way.node_refs.size(); ++i) {
          ids[i] = way.node_refs[i].ref();
        }
      }
      osm_id_to_way_[way.id] = idx;
    }

    return result;
  }

  template <typename Members, typename Tags>
  polygon_area assemble_area(std::int64_t const id,
                             Members&& members,
                             Tags&& tags) {
    auto a = polygon_area{id};
    if (!is_area(tags)) {
      return a;
    }

    auto assemble = make_assembly();
    auto r = relation{id, members};

    // Materialize temporary `way`s for each member so the assembler (which
    // takes `std::vector<way const*>`) sees stable pointers. Node ids are 0
    // unless `debug_` keeps them in `way_node_ids_`.
    auto ways_storage = std::vector<way>{};
    auto ways = std::vector<way const*>{};
    if (auto const mit = mp_index_.find(id); mit != mp_index_.end()) {
      auto const& mp = mp_vec_[mit->second];
      ways_storage.reserve(mp.ways_refs_.size());
      for (auto const osm_id : mp.ways_refs_) {
        auto const it = osm_id_to_way_.find(osm_id);
        if (it == osm_id_to_way_.end()) {
          continue;
        }
        auto const w_idx = it->second;
        auto const locations = way_locations_[w_idx];
        auto refs = std::vector<node_ref>{};
        refs.reserve(locations.size());
        for (auto i = 0U; i != locations.size(); ++i) {
          refs.push_back(
              node_ref{debug_ ? way_node_ids_[w_idx][i] : 0, locations[i]});
        }
        ways_storage.push_back(way{way_to_id_[w_idx], std::move(refs)});
      }
      ways.reserve(ways_storage.size());
      for (auto const& w : ways_storage) {
        ways.push_back(&w);
      }
      utl::sort(ways, [](way const* a, way const* b) { return a->id < b->id; });
    }

    assemble.assembling_area_from_relation(r, ways, a);

    all_stats_ += a.pa_stats;

    return a;
  }

  assembly make_assembly() const {
    return debug_ ? assembly{&std::cerr, true} : assembly{};
  }

  area_stats all_stats_{};
  bool assemble_way_polygons_{false};
  bool debug_{false};

  std::mutex mp_vec_mtx_;
  std::vector<multi_polygon> mp_vec_{};
  hash_map<object_id_type, std::size_t> mp_index_{};  // relation id -> mp_vec_

  std::mutex ways_vec_mtx_;
  vecvec<way_idx_t, location> way_locations_{};
  vecvec<way_idx_t, object_id_type> way_node_ids_{};  // debug only
  vector_map<way_idx_t, object_id_type> way_to_id_{};
  hash_map<object_id_type, way_idx_t> osm_id_to_way_{};
  std::atomic_uint64_t count_non_areas_{0};

  // Sorted, deduplicated way ids that area relations reference; only these are
  // stored in pass 2. Empty + `members_indexed_ == false` means "store all".
  std::vector<object_id_type> member_way_ids_{};
  bool members_indexed_{false};
};

}  // namespace osm