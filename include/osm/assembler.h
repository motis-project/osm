#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "fmt/ostream.h"

#include "utl/helpers/algorithm.h"

#include "assembler/assembler_types.h"
#include "assembler/state.h"

namespace osm {

struct assembly {
  explicit assembly(std::ostream* pr = nullptr, bool debug_flag = false)
      : state_{pr, debug_flag} {}

  // Definition of helper functions as members:
  std::uint32_t add_new_ring_complex(slocation const& node);
  std::uint32_t add_new_ring(slocation const& node);
  bool create_rings_complex_case();
  node_ref_segment* get_next_segment(location const& location);
  std::vector<location_to_ring_map> create_location_to_ring_map(
      open_ring_its_type& open_ring_its);
  void merge_two_rings(open_ring_its_type& open_ring_its,
                       location_to_ring_map const& m1,
                       location_to_ring_map const& m2);
  bool try_to_merge(open_ring_its_type& open_ring_its);
  proto_ring* find_enclosing_ring(node_ref_segment* segment);
  void find_inner_outer_complex();
  void find_inner_outer_complex(proto_ring* ring);
  bool join_connected_rings(open_ring_its_type& open_ring_its);

  bool create_rings();
  bool create_area(polygon_area& out_buffer);
  bool assembling_area_from_way(way const& w, polygon_area& out_buffer);

  /**
   * Assemble an area from the given relation and its members.
   * The resulting area is put into the out_buffer.
   * @returns false if there was some kind of error building the
   *          area(s), true otherwise.
   */
  template <typename Members>
  bool assembling_area_from_relation(relation<Members> const& relation,
                                     std::vector<way const*> const& ways,
                                     polygon_area& out_buffer) {
    assert(relation.members().size() >= ways.size());

    if (relation.members().empty()) {
      ++state_.stats.no_way_in_mp_relation;
      return false;
    }
    ++state_.stats.from_relations;
    state_.stats.invalid_locations =
        state_.segment_list.extract_segments_from_ways(
            state_.problem_reporter, state_.stats.duplicate_nodes,
            state_.stats.duplicate_ways, relation, ways);

    if (state_.stats.invalid_locations > 0) {
      return false;
    }
    state_.stats.member_ways = ways.size();

    if (state_.stats.member_ways == 1) {
      ++state_.stats.single_way_in_mp_relation;
    }

    if (state_.debug) {
      fmt::println(
          std::cerr,
          "\nAssembling relation {}containing {} way members with {} nodes",
          relation.id, ways.size(), state_.segment_list.size());
    }
    auto const okay = create_area(out_buffer);
    auto const found_it =
        utl::find(state_.segment_list.relations_missing_ways, relation.id);
    if (found_it != state_.segment_list.relations_missing_ways.end()) {
      out_buffer.missing_flag = true;
    }
    if (okay) {
      out_buffer.pa_stats = state_.stats;
    }
    if (state_.debug) {
      fmt::println(std::cerr, "Done: {}", relation.id);
      state_.stats.print_stats();
    }
    return okay;
  }

  state state_;
};

}  // namespace osm
