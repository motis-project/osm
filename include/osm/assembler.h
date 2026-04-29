#pragma once

#include <cassert>
#include <iostream>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "assembler/assembler_types.h"
#include "assembler/state.h"

namespace assembler {
struct assembly {

  State state_;
  assembly(std::ostream* pr = nullptr, bool debug_flag = false)
      : state_(pr, debug_flag) {}

  // Definition of helper functions as members:
  uint32_t add_new_ring_complex(const slocation& node);
  std::uint32_t add_new_ring(const slocation& node);
  bool create_rings_complex_case();
  NodeRefSegment* get_next_segment(const osm::Location& location);
  std::vector<location_to_ring_map> create_location_to_ring_map(
      open_ring_its_type& open_ring_its);
  void merge_two_rings(open_ring_its_type& open_ring_its,
                       const location_to_ring_map& m1,
                       const location_to_ring_map& m2);
  bool try_to_merge(open_ring_its_type& open_ring_its);
  ProtoRing* find_enclosing_ring(NodeRefSegment* segment);
  void find_inner_outer_complex();
  void find_inner_outer_complex(ProtoRing* ring);
  bool join_connected_rings(open_ring_its_type& open_ring_its);

  bool create_rings() {
    state_.stats.nodes += state_.segment_list.size();
    // Sort the list of segments (from left to right and bottom
    // to top).
    state_.segment_list.sort();
    // Remove duplicate segments. Removal is in pairs, so if there
    // are two identical segments, they will both be removed. If
    // there are three, two will be removed and one remains.
    state_.segment_list.erase_duplicate_segments(
        &state_.problem_reporter, state_.stats.duplicate_segments,
        state_.stats.overlapping_segments);
    // If there are no segments left at this point, this isn't
    // a valid area.
    if (state_.segment_list.empty()) {
      if (state_.debug) {
        std::cerr << "  No segments left\n";
      }
      return false;
    }
    // Now we look for segments crossing each other. If there are
    // any, the multipolygon is invalid.
    // In the future this could be improved by trying to fix those
    // cases.
    state_.stats.intersections =
        state_.segment_list.find_intersections(&state_.problem_reporter);
    if (state_.stats.intersections) {
      return false;
    }
    // slocations: An ordered list of locations of both endpoints
    // of all segments with pointers back to the segments. We will
    // use this list later to quickly find which segment(s) fits
    // onto a known segment.
    state_.slocations.reserve(state_.segment_list.size() * 2);
    // static_cast is okay here: The 32bit limit is way past
    // anything that makes sense here and even if there are
    // 2^32 segments here, it would simply not go through
    // all of them not building the multipolygon correctly.
    assert(state_.segment_list.size() < std::numeric_limits<uint32_t>::max());
    for (uint32_t n = 0; n < static_cast<uint32_t>(state_.segment_list.size());
         ++n) {
      state_.slocations.emplace_back(n, false);
      state_.slocations.emplace_back(n, true);
    }
    std::stable_sort(state_.slocations.begin(), state_.slocations.end(),
                     [this](const slocation& lhs, const slocation& rhs) {
                       return lhs.location(state_.segment_list) <
                              rhs.location(state_.segment_list);
                     });
    // Find all locations where more than two segments start or
    // end. We call those "split" locations. If there are any
    // "spike" segments found while doing this, we know the area
    // geometry isn't valid and return.
    /**
     * If there are any open rings found along the way, they are reported
     * and the function returns false.
     */
    bool found_open_rings = false;
    osm::Location previous_location;
    for (auto it = state_.slocations.cbegin(); it != state_.slocations.cend();
         ++it) {
      const osm::NodeRef& nr = it->node_ref(state_.segment_list);
      const osm::Location& loc = nr.location();
      if (std::next(it) == state_.slocations.cend() ||
          loc != std::next(it)->location(state_.segment_list)) {
        if (state_.debug) {
          std::cerr << " Found open ring at " << nr.ref() << "\n";
        }
        const auto& segment = state_.segment_list[it->item];
        state_.problem_reporter.report_ring_not_closed(nr, segment.way());
        ++state_.stats.open_rings;
      } else {
        if (loc == previous_location &&
            (state_.split_locations.empty() ||
             state_.split_locations.back() != previous_location)) {
          state_.split_locations.push_back(previous_location);
        }
        ++it;
        if (it == state_.slocations.end()) {
          break;
        }
      }
      previous_location = loc;
    }
    found_open_rings = state_.stats.open_rings != 0;
    if (found_open_rings) {
      return false;
    }
    // Now report all split locations to the problem reporter.
    state_.stats.touching_rings += state_.split_locations.size();
    if (!state_.split_locations.empty()) {
      if (state_.debug) {
        std::cerr << "  Found split locations:\n";
      }
      for (const auto& location : state_.split_locations) {
        auto it = std::lower_bound(
            state_.slocations.cbegin(), state_.slocations.cend(), slocation{},
            [this, &location](const slocation& lhs, const slocation& rhs) {
              return lhs.location(state_.segment_list, location) <
                     rhs.location(state_.segment_list, location);
            });
        assert(it != state_.slocations.cend());
        const osm::object_id_type id = it->node_ref(state_.segment_list).ref();
        state_.problem_reporter.report_touching_ring(id, location);
        if (state_.debug) {
          std::cerr << "    " << location.x() << "," << location.y() << "\n";
        }
      }
    }
    // From here on we use two different algorithms depending on
    // whether there were any split locations or not. If there
    // are no splits, we use the faster "simple algorithm", if
    // there are, we use the slower "complex algorithm".
    if (state_.split_locations.empty()) {
      if (state_.debug) {
        std::cerr << " No split locations -> using simple algorithm\n";
      }
      ++state_.stats.area_simple_case;
      // create_rings_simple_case:
      auto count_remaining = state_.segment_list.size();
      for (const slocation& sl : state_.slocations) {
        const NodeRefSegment& segment = state_.segment_list[sl.item];
        if (!segment.is_done()) {
          count_remaining -= add_new_ring(sl);
          if (count_remaining == 0) {
            break;
          }
        }
      }  // create_rings_simple_case - finished
    } else if (state_.split_locations.size() > max_split_locations_) {
      if (state_.debug) {
        std::cerr << " Ignoring polygon with " << state_.split_locations.size()
                  << " split locations (>" << max_split_locations_ << ")\n";
      }
      return false;
    } else {
      if (state_.debug) {
        std::cerr << " Found " << state_.split_locations.size()
                  << " split locations -> using complex algorithm\n";
      }
      ++state_.stats.area_touching_rings_case;
      if (!create_rings_complex_case()) {
        return false;
      }
    }
    // If the assembler was so configured, now check whether the
    // member roles are correctly tagged. --> check always
    // check_inner_outer_roles:
    if (state_.debug) {
      std::cerr << "    Checking inner/outer roles\n";
    }
    int count_segments_for_debug = 0;
    std::unordered_map<const osm::Way*, const ProtoRing*> way_rings;
    std::unordered_set<const osm::Way*> ways_in_multiple_rings;
    for (const ProtoRing& ring : state_.rings) {
      for (const auto& segment : ring.segments()) {
        count_segments_for_debug++;
        assert(segment->way());
        if (!segment->role_empty() &&
            (ring.is_outer() ? !segment->role_outer()
                             : !segment->role_inner())) {
          ++state_.stats.wrong_role;
          if (state_.debug) {
            std::cerr << " Segment: " << count_segments_for_debug
                      << " from way " << segment->way()->id << " has role '"
                      << segment->role_name() << "', but should have role '"
                      << (ring.is_outer() ? "outer" : "inner") << "'\n ";
          }
          if (ring.is_outer()) {
            state_.problem_reporter.report_role_should_be_outer(
                segment->way()->id, segment->first().location(),
                segment->second().location());
          } else {
            state_.problem_reporter.report_role_should_be_inner(
                segment->way()->id, segment->first().location(),
                segment->second().location());
          }
        }
        auto& r = way_rings[segment->way()];
        if (!r) {
          r = &ring;
        } else if (r != &ring) {
          ways_in_multiple_rings.insert(segment->way());
        }
      }
      count_segments_for_debug = 0;
    }
    for (const osm::Way* way :
         ways_in_multiple_rings) {  // NOLINT(bugprone - nondeterministic -
                                    // pointer - iteration - order)
      ++state_.stats.ways_in_multiple_rings;
      if (state_.debug) {
        std::cerr << " Way " << way->id << " is in multiple rings\n ";
      }
      state_.problem_reporter.report_way_in_multiple_rings(*way);
    }
    // check_inner_outer_roles - finished

    state_.stats.outer_rings =
        std::count_if(state_.rings.cbegin(), state_.rings.cend(),
                      [](const ProtoRing& ring) { return ring.is_outer(); });
    state_.stats.inner_rings = state_.rings.size() - state_.stats.outer_rings;
    return true;
  }

  /**
   * Assembles area objects from closed ways or multipolygon relations
   * and their members.
   */
  bool create_area(polygon_area& out_buffer) {
    const bool area_okay = create_rings();
    if (area_okay) {
      std::vector<area_pair> area;
      for (const ProtoRing& ring : state_.rings) {
        if (!ring.is_outer()) {
          continue;
        }
        std::vector<osm::NodeRef> area_part;
        std::vector<std::int64_t> offsets;
        offsets.push_back(area_part.size());
        area_part.emplace_back(ring.get_node_ref_start());
        for (const auto& segment : ring.segments()) {
          area_part.emplace_back(segment->stop());
        }
        for (const ProtoRing* inner : ring.inner_rings()) {
          offsets.push_back(area_part.size());
          area_part.emplace_back(inner->get_node_ref_start());
          for (const auto& segment : inner->segments()) {
            area_part.emplace_back(segment->stop());
          }
        }
        area.emplace_back(area_part, offsets);
      }
      out_buffer.valid = area_okay;
      out_buffer.area = std::move(area);
    }
    return area_okay;
  }

  /**
   * Assemble an area from the given way.
   * The resulting area is put into the out_buffer.
   * @returns false if there was some kind of error building the
   *          area, true otherwise.
   */
  bool assembling_area_from_way(const osm::Way& way, polygon_area& out_buffer) {
    // Ignore (but count) ways without segments.
    if (way.nodes().size() < 2) {
      ++state_.stats.short_ways;
      return false;
    }

    if (!way.ends_have_same_id()) {
      ++state_.stats.duplicate_nodes;
      if (state_.problem_reporter.report) {
        state_.problem_reporter.report_duplicate_node(
            way.nodes().front().ref(), way.nodes().back().ref(),
            way.nodes().front().location());
      }
    }

    ++state_.stats.from_ways;
    state_.stats.invalid_locations =
        state_.segment_list.extract_segments_from_way(
            state_.problem_reporter, state_.stats.duplicate_nodes, way);

    if (state_.stats.invalid_locations > 0) {
      return false;
    }
    if (state_.debug) {
      std::cerr << "\nAssembling way " << way.id << " containing "
                << state_.segment_list.size() << " nodes\n";
    }
    const bool okay = create_area(out_buffer);
    if (state_.debug) {
      std::cerr << "Done: " << way.id << std::endl;
      state_.stats.print_stats();
    }
    return okay;
  }

  /**
   * Assemble an area from the given relation and its members.
   * The resulting area is put into the out_buffer.
   * @returns false if there was some kind of error building the
   *          area(s), true otherwise.
   */
  template <typename Members>
  bool assembling_area_from_relation(const osm::Relation<Members>& relation,
                                     const std::vector<const osm::Way*>& ways,
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
      std::cerr << "\nAssembling relation " << relation.id << "containing "
                << ways.size() << " way members with "
                << state_.segment_list.size() << " nodes\n";
    }
    const bool okay = create_area(out_buffer);
    auto found_it = std::find(
        state_.segment_list.relations_missing_ways.begin(),
        state_.segment_list.relations_missing_ways.end(), relation.id);
    if (found_it != state_.segment_list.relations_missing_ways.end()) {
      out_buffer.missing_flag = true;
    }
    if (okay) {
      out_buffer.pa_stats = state_.stats;
    }
    if (state_.debug) {
      std::cerr << "Done: " << relation.id << std::endl;
      state_.stats.print_stats();
    }
    return okay;
  }

};  // class assembly

}  // namespace assembler
