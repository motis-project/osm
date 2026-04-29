#pragma once

#include <cassert>
#include <iterator>
#include <list>

#include "assembler_types.h"
#include "iter.h"
#include "osm/assembler.h"
#include "osm/types.h"

namespace assembler {

/*
 *  helper functions:
 */

inline NodeRefSegment* assembly::get_next_segment(
    const osm::Location& location) {
  auto it = std::lower_bound(
      state_.slocations.begin(), state_.slocations.end(), slocation{},
      [&location, this](const slocation& lhs, const slocation& rhs) {
        return lhs.location(state_.segment_list, location) <
               rhs.location(state_.segment_list, location);
      });

  assert(it != state_.slocations.end());
  if (state_.segment_list[it->item].is_done()) {
    ++it;
  }
  assert(it != state_.slocations.end());
  assert(!state_.segment_list[it->item].is_done());
  return &state_.segment_list[it->item];
}

inline void remove_duplicates(std::vector<rings_stack_element>& outer_rings) {
  while (true) {
    const auto it = std::adjacent_find(outer_rings.begin(), outer_rings.end());
    if (it == outer_rings.end()) {
      return;
    }
    outer_rings.erase(it, std::next(it, 2));
  }
}

inline std::vector<location_to_ring_map> assembly::create_location_to_ring_map(
    open_ring_its_type& open_ring_its) {
  std::vector<location_to_ring_map> xrings;
  xrings.reserve(open_ring_its.size() * 2);

  int count_for_debug = 0;
  for (auto it = open_ring_its.begin(); it != open_ring_its.end(); ++it) {
    if (state_.debug) {
      ++count_for_debug;
      std::cerr << "      " << count_for_debug << '\n';
    }
    xrings.emplace_back((*it)->get_node_ref_start().location(), it, true);
    xrings.emplace_back((*it)->get_node_ref_stop().location(), it, false);
  }
  std::stable_sort(xrings.begin(), xrings.end());
  return xrings;
}

inline void assembly::merge_two_rings(open_ring_its_type& open_ring_its,
                                      const location_to_ring_map& m1,
                                      const location_to_ring_map& m2) {
  const auto r1 = *m1.ring_it;
  const auto r2 = *m2.ring_it;

  if (r1->get_node_ref_stop().location() ==
      r2->get_node_ref_start().location()) {
    r1->join_forward(*r2);
  } else if (r1->get_node_ref_stop().location() ==
             r2->get_node_ref_stop().location()) {
    r1->join_backward(*r2);
  } else if (r1->get_node_ref_start().location() ==
             r2->get_node_ref_start().location()) {
    r1->reverse();
    r1->join_forward(*r2);
  } else if (r1->get_node_ref_start().location() ==
             r2->get_node_ref_stop().location()) {
    r1->reverse();
    r1->join_backward(*r2);
  } else {
    assert(false);
  }
  open_ring_its.erase(
      std::find(open_ring_its.begin(), open_ring_its.end(), r2));
  state_.rings.erase(r2);

  if (r1->closed()) {
    open_ring_its.erase(
        std::find(open_ring_its.begin(), open_ring_its.end(), r1));
  }
}

inline bool assembly::try_to_merge(open_ring_its_type& open_ring_its) {
  if (open_ring_its.empty()) {
    return false;
  }
  if (state_.debug) {
    std::cerr << "    Trying to merge " << open_ring_its.size()
              << " open rings (try_to_merge)\n";
  }

  const auto xrings = create_location_to_ring_map(open_ring_its);
  auto it = xrings.cbegin();
  while (it != xrings.cend()) {
    it = std::adjacent_find(it, xrings.cend());
    if (it == xrings.cend()) {
      return false;
    }
    auto after = std::next(it, 2);
    if (after == xrings.cend() || after->location != it->location) {
      if (state_.debug) {
        std::cerr << "      Merging two rings\n";
      }
      merge_two_rings(open_ring_its, *it, *std::next(it));
      return true;
    }
    while (it != xrings.cend() && it->location == after->location) {
      ++it;
    }
  }
  return false;
}

inline ProtoRing* assembly::find_enclosing_ring(NodeRefSegment* segment) {
  if (state_.debug) {
    std::cerr << "    Looking for ring enclosing \n";
  }

  const auto location = segment->first().location();
  const auto end_location = segment->second().location();

  while (segment->first().location() == location) {
    if (segment == &state_.segment_list.back()) {
      break;
    }
    ++segment;
  }

  int nesting = 0;
  std::vector<rings_stack_element> outer_rings;
  while (segment >= &state_.segment_list.front()) {
    if (!segment->is_direction_done()) {
      --segment;
      continue;
    }
    if (state_.debug) {
      std::cerr << "      Checking against " << segment << "\n";
    }
    const osm::Location& a = segment->first().location();
    const osm::Location& b = segment->second().location();

    if (segment->first().location() == location) {
      const std::int64_t ax = a.x();
      const std::int64_t bx = b.x();
      const std::int64_t lx = end_location.x();
      const std::int64_t ay = a.y();
      const std::int64_t by = b.y();
      const std::int64_t ly = end_location.y();
      const auto z = ((bx - ax) * (ly - ay)) - ((by - ay) * (lx - ax));
      if (state_.debug) {
        std::cerr << "      Segment z=" << z << '\n';
      }
      if (z > 0) {
        nesting += segment->is_reverse() ? -1 : 1;
        if (state_.debug) {
          std::cerr << "        Segment is below (nesting=" << nesting << ")\n";
        }
        if (segment->ring()->is_outer()) {
          if (state_.debug) {
            std::cerr << "        Segment belongs to outer ring (y=" << a.y()
                      << " ring=" << segment->ring() << ")\n";
          }
          outer_rings.emplace_back(a.y(), segment->ring());
        }
      }
    } else if (a.x() <= location.x() && location.x() < b.x()) {
      if (state_.debug) {
        std::cerr << "        Is in x range\n";
      }

      const std::int64_t ax = a.x();
      const std::int64_t bx = b.x();
      const std::int64_t lx = location.x();
      const std::int64_t ay = a.y();
      const std::int64_t by = b.y();
      const std::int64_t ly = location.y();
      const auto z = ((bx - ax) * (ly - ay)) - ((by - ay) * (lx - ax));

      if (z >= 0) {
        nesting += segment->is_reverse() ? -1 : 1;
        if (state_.debug) {
          std::cerr << "        Segment is below (nesting=" << nesting << ")\n";
        }
        if (segment->ring()->is_outer()) {
          const double y = static_cast<double>(ay) +
                           (static_cast<double>((by - ay) * (lx - ax)) /
                            static_cast<double>(bx - ax));
          if (state_.debug) {
            std::cerr << "        Segment belongs to outer ring (y=" << y
                      << " ring=" << segment->ring() << ")\n";
          }
          outer_rings.emplace_back(y, segment->ring());
        }
      }
    }
    --segment;
  }

  if (nesting % 2 == 0) {
    if (state_.debug) {
      std::cerr << "    Decided that this is an outer ring\n";
    }
    return nullptr;
  }
  if (state_.debug) {
    std::cerr << "    Decided that this is an inner ring\n";
  }
  assert(!outer_rings.empty());
  std::stable_sort(outer_rings.rbegin(), outer_rings.rend());
  if (state_.debug) {
    for (const auto& o : outer_rings) {
      std::cerr << "        y=" << o.y()
                << std::endl;  // " " << o.ring() << "\n";
    }
  }
  remove_duplicates(outer_rings);
  if (state_.debug) {
    std::cerr << "      after remove duplicates:\n";
    for (const auto& o : outer_rings) {
      std::cerr << "        y=" << o.y()
                << std::endl;  //" " << o.ring() << "\n";
    }
  }

  assert(!outer_rings.empty());
  return outer_rings.front().ring_ptr();
}

inline void assembly::find_inner_outer_complex(ProtoRing* ring) {
  ProtoRing* outer_ring = find_enclosing_ring(ring->min_segment());
  if (outer_ring) {
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  }
  ring->fix_direction();
  ring->mark_direction_done();
}
inline void assembly::find_inner_outer_complex() {
  if (state_.debug) {
    std::cerr << "  Finding inner/outer rings\n";
  }
  std::vector<ProtoRing*> closed_rings;
  closed_rings.reserve(state_.rings.size());
  for (auto& ring : state_.rings) {
    if (ring.closed()) {
      closed_rings.push_back(&ring);
    }
  }
  if (closed_rings.empty()) {
    return;
  }
  std::stable_sort(closed_rings.begin(), closed_rings.end(),
                   [](ProtoRing* a, ProtoRing* b) {
                     return a->min_segment() < b->min_segment();
                   });
  closed_rings.front()->fix_direction();
  closed_rings.front()->mark_direction_done();
  if (state_.debug) {
    std::cerr << "    First ring is outer: " << closed_rings.front() << "\n";
  }
  int count_for_debug = 0;
  for (auto it = std::next(closed_rings.begin()); it != closed_rings.end();
       ++it) {
    ++count_for_debug;
    if (state_.debug) {
      std::cerr << "    Checking (at min segment " << (*it)->min_segment()
                << ") ring " << count_for_debug << std::endl;  //**it << "\n";
    }
    find_inner_outer_complex(*it);
    if (state_.debug) {
      std::cerr << "    Ring is " << ((*it)->is_outer() ? "OUTER: " : "INNER: ")
                << count_for_debug << std::endl;  //**it << "\n";
    }
  }
}

inline void find_candidates(std::vector<candidate>& candidates,
                            std::vector<osm::Location>& loc_done,
                            const std::vector<location_to_ring_map>& xrings,
                            const candidate& cand,
                            unsigned depth = 0,
                            bool debug = false) {
  if (depth > max_depth) {
    throw std::exception{};
  }

  if (debug) {
    std::cerr << "      find_candidates sum=" << cand.sum
              << " start=" << cand.start_location.x() << ","
              << cand.start_location.y() << " stop=" << cand.stop_location.x()
              << "," << cand.stop_location.y() << "\n";
  }

  const auto connections =
      make_range(std::equal_range(xrings.cbegin(), xrings.cend(),
                                  location_to_ring_map{cand.stop_location}));

  assert(connections.begin() != connections.end());

  assert(!cand.rings.empty());
  const ProtoRing* ring_leading_here = &cand.rings.back().first.ring();
  for (const location_to_ring_map& m : connections) {
    const ProtoRing& ring = m.ring();

    if (&ring != ring_leading_here) {
      // if (debug) {
      //   std::cerr << "        next possible connection: ";
      //   ring.print(std::cerr);
      //   std::cerr << (m.start ? "" : " reverse") << "\n";
      // }

      candidate c = cand;
      if (m.start) {
        c.rings.emplace_back(m, false);
        c.stop_location = ring.get_node_ref_stop().location();
        c.sum += ring.sum();
      } else {
        c.rings.emplace_back(m, true);
        c.stop_location = ring.get_node_ref_start().location();
        c.sum -= ring.sum();
      }
      if (c.closed()) {
        if (debug) {
          std::cerr << "          found candidate\n";
        }

        if (candidates.empty()) {
          candidates.push_back(c);
        } else if (candidates.size() == 1) {
          // add new candidate to vector, keep sorted
          if (std::abs(c.sum) < std::abs(candidates.front().sum)) {
            candidates.insert(candidates.begin(), c);
          } else {
            candidates.push_back(c);
          }
        } else {
          // add new candidate if it has either smallest or largest area
          if (std::abs(c.sum) < std::abs(candidates.front().sum)) {
            candidates.front() = c;
          } else if (std::abs(c.sum) > std::abs(candidates.back().sum)) {
            candidates.back() = c;
          }
        }
      } else if (std::find(loc_done.cbegin(), loc_done.cend(),
                           c.stop_location) == loc_done.cend()) {
        if (debug) {
          std::cerr << "          recurse... (depth=" << depth
                    << " candidates.size=" << candidates.size()
                    << " loc_done.size=" << loc_done.size() << ")\n";
        }
        loc_done.push_back(c.stop_location);
        find_candidates(candidates, loc_done, xrings, c, depth + 1, debug);
        assert(!loc_done.empty() && loc_done.back() == c.stop_location);
        loc_done.pop_back();
        if (debug) {
          std::cerr << "          ...back\n";
        }
      } else if (debug) {
        std::cerr << "          loop found\n";
      }
    }
  }
}
// helper functions end

/**
 * If there are multiple open rings and multiple ways to join them,
 * this function is called. It will take the first open ring and
 * try recursively all ways of closing it. Of all the candidates
 * the one with the smallest area is chosen and closed. If it
 * can't close this ring, an error is reported and the function
 * returns false.
 */
inline bool assembly::join_connected_rings(open_ring_its_type& open_ring_its) {
  assert(!open_ring_its.empty());
  if (state_.debug) {
    std::cerr << "    Trying to merge " << open_ring_its.size()
              << " open rings (join_connected_rings)\n";
  }
  std::vector<location_to_ring_map> xrings =
      create_location_to_ring_map(open_ring_its);

  const auto ring_min = std::min_element(
      xrings.begin(), xrings.end(),
      [](const location_to_ring_map& lhs, const location_to_ring_map& rhs) {
        return lhs.ring().min_segment() < rhs.ring().min_segment();
      });

  find_inner_outer_complex();
  ProtoRing* outer_ring = find_enclosing_ring(ring_min->ring().min_segment());
  const bool ring_min_is_outer = !outer_ring;
  if (state_.debug) {
    std::cerr << "  Open ring is " << (ring_min_is_outer ? "outer" : "inner")
              << " ring\n";
  }
  for (auto& ring : state_.rings) {
    ring.reset();
  }

  const candidate cand{*ring_min, false};
  std::vector<osm::Location> loc_done;
  loc_done.push_back(cand.stop_location);
  std::vector<candidate> candidates;

  try {
    find_candidates(candidates, loc_done, xrings, cand, state_.debug);
  } catch (std::exception e) {
    if (state_.debug) {
      std::cerr << "Exceeded max depth (" << max_depth << ")\n";
    }
    return false;
  }

  if (candidates.empty()) {
    if (state_.debug) {
      std::cerr << "    Found no candidates\n";
    }
    if (!open_ring_its.empty()) {
      std::cerr << "But open rings remain! \n";
      ++state_.stats.open_rings;
      // if (problem_reporter.report) {
      //   for (auto& it : open_ring_its) {
      //     m_config.problem_reporter->report_ring_not_closed(
      //         it->get_node_ref_start(), nullptr);
      //     m_config.problem_reporter->report_ring_not_closed(
      //         it->get_node_ref_stop(), nullptr);
      //   }
      // }
    }
    return false;
  }

  if (state_.debug) {
    std::cerr << "    Found candidates:\n";
    for (const auto& c : candidates) {
      std::cerr << "      sum=" << c.sum << "\n";
      // for (const auto& ring : c.rings) {
      //   std::cerr << "        ";
      //   ring.first.ring().print(std::cerr);
      //   std::cerr << (ring.second ? " reverse" : "") << "\n";
      // }
    }
  }

  // Find the candidate with the smallest/largest area
  const auto chosen_cand =
      ring_min_is_outer ? candidates.front() : candidates.back();

  if (state_.debug) {
    std::cerr << "    Decided on: sum=" << chosen_cand.sum << "\n";
    // for (const auto& ring : chosen_cand.rings) {
    //   std::cerr << "        ";
    //   ring.first.ring().print(std::cerr);
    //   std::cerr << (ring.second ? " reverse" : "") << "\n";
    // }
  }

  // Join all (open) rings in the candidate to get one closed ring.
  assert(chosen_cand.rings.size() > 1);
  const auto& first_ring = chosen_cand.rings.front().first;
  const ProtoRing& remaining_ring = first_ring.ring();
  for (auto it = std::next(chosen_cand.rings.begin());
       it != chosen_cand.rings.end(); ++it) {
    merge_two_rings(open_ring_its, first_ring, it->first);
  }

  if (state_.debug) {
    std::cerr << "    Merged \n";  // to ";
    // remaining_ring.print(std::cerr);
    // std::cerr << '\n';
  }
  return true;
}

inline uint32_t assembly::add_new_ring_complex(const slocation& node) {
  NodeRefSegment* segment = &state_.segment_list[node.item];
  assert(!segment->is_done());
  if (state_.debug) {
    std::cerr << "  Starting new ring at location "
              << node.location(state_.segment_list).x() << ","
              << node.location(state_.segment_list).y() << " with segment "
              << segment << "\n";
  }
  if (node.reverse) {
    segment->reverse();
  }
  state_.rings.emplace_back(segment);
  ProtoRing* ring = &state_.rings.back();

  const osm::Location& first_location = node.location(state_.segment_list);
  osm::Location last_location = segment->stop().location();

  auto is_split_location = [&](osm::Location loc) {
    return std::find(state_.split_locations.cbegin(),
                     state_.split_locations.cend(),
                     loc) != state_.split_locations.cend();
  };

  uint32_t nodes = 1;
  while (first_location != last_location && !is_split_location(last_location)) {
    ++nodes;
    NodeRefSegment* next_segment = assembly::get_next_segment(last_location);
    if (next_segment->start().location() != last_location) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    if (state_.debug) {
      std::cerr << "    Next segment is " << next_segment << "\n";
    }
    last_location = next_segment->stop().location();
  }
  if (state_.debug) {
    if (first_location == last_location) {
      std::cerr << "    Completed ring: " << ring << "\n";
    } else {
      std::cerr << "    Completed partial ring: " << ring << "\n";
    }
  }
  return nodes;
}

inline std::uint32_t assembly::add_new_ring(const slocation& node) {
  NodeRefSegment* segment = &state_.segment_list[node.item];
  assert(!segment->is_done());
  if (state_.debug) {
    std::cerr << "  Starting new ring at location "
              << node.location(state_.segment_list).x() << ","
              << node.location(state_.segment_list).y() << " with segment "
              << segment << "\n";
  }
  if (node.reverse) {
    segment->reverse();
  }

  ProtoRing* outer_ring = nullptr;
  if (segment != &state_.segment_list.front()) {
    outer_ring = find_enclosing_ring(segment);
  }
  segment->mark_direction_done();
  state_.rings.emplace_back(segment);
  ProtoRing* ring = &state_.rings.back();
  if (outer_ring) {
    if (state_.debug) {
      std::cerr << "    This is an inner ring:\n";
    }
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  } else if (state_.debug) {
    std::cerr << "    This is an outer ring\n";
  }

  const osm::Location& first_location = node.location(state_.segment_list);
  osm::Location last_location = segment->stop().location();

  uint32_t nodes = 1;
  while (first_location != last_location) {
    ++nodes;
    NodeRefSegment* next_segment = get_next_segment(last_location);
    next_segment->mark_direction_done();
    if (next_segment->start().location() != last_location) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    if (state_.debug) {
      std::cerr << "    Next segment is " << next_segment->first_noderef_.ref()
                << "\n";
    }
    last_location = next_segment->stop().location();
  }

  ring->fix_direction();

  if (state_.debug) {
    std::cerr << "    Completed ring \n";
  }
  return nodes;
}

inline bool assembly::create_rings_complex_case() {
  // First create all the (partial) rings starting at the split locations
  auto count_remaining = state_.segment_list.size();
  for (const osm::Location& location : state_.split_locations) {
    const auto locs = make_range(std::equal_range(
        state_.slocations.begin(), state_.slocations.end(), slocation{},
        [&location, this](const slocation& lhs, const slocation& rhs) {
          return lhs.location(state_.segment_list, location) <
                 rhs.location(state_.segment_list, location);
        }));
    for (auto& loc : locs) {
      if (!state_.segment_list[loc.item].is_done()) {
        count_remaining -= add_new_ring_complex(loc);
        if (count_remaining == 0) {
          break;
        }
      }
    }
  }
  // Now find all the rest of the rings (ie not starting at split
  // locations)
  if (count_remaining > 0) {
    for (const slocation& sl : state_.slocations) {
      const NodeRefSegment& segment = state_.segment_list[sl.item];
      if (!segment.is_done()) {
        count_remaining -= add_new_ring_complex(sl);
        if (count_remaining == 0) {
          break;
        }
      }
    }
  }

  auto there_are_open_rings = [&]() {
    return std::any_of(state_.rings.cbegin(), state_.rings.cend(),
                       [](const ProtoRing& ring) { return !ring.closed(); });
  };

  // Now all segments are in exactly one (partial) ring.
  // If there are open rings, try to join them to create closed
  // rings.
  if (there_are_open_rings()) {
    ++state_.stats.area_really_complex_case;

    open_ring_its_type open_ring_its;
    for (auto it = state_.rings.begin(); it != state_.rings.end(); ++it) {
      if (!it->closed()) {
        open_ring_its.push_back(it);
      }
    }

    while (!open_ring_its.empty()) {
      if (state_.debug) {
        std::cerr << "There are " << open_ring_its.size() << " open rings\n";
      }
      while (try_to_merge(open_ring_its)) {
        // intentionally left blank
      }
      if (!open_ring_its.empty()) {
        if (state_.debug) {
          std::cerr << "  After joining obvious cases there are still "
                    << open_ring_its.size() << " open rings\n";
        }
        if (!join_connected_rings(open_ring_its)) {
          return false;
        }
      }
    }

    if (state_.debug) {
      std::cerr << "  Joined all open rings\n";
    }
  }

  // Now all rings are complete.
  find_inner_outer_complex();

  return true;
}

}  // namespace assembler