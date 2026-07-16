#include "osm/assembler.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <iterator>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "fmt/ostream.h"

#include "utl/helpers/algorithm.h"
#include "utl/verify.h"

#include "osm/assembler/assembler_types.h"
#include "osm/assembler/iter.h"
#include "osm/assembler/state.h"
#include "osm/types.h"

// dbg macro (used only by this TU; include after the headers so callers
// of assembler.h don't see it)
#define dbg(...)                              \
  do {                                        \
    if (state_.debug) {                       \
      fmt::println(std::cerr, __VA_ARGS__);   \
    }                                         \
  } while (0)

namespace osm {

/*
 *  helper functions:
 */

node_ref_segment* assembly::get_next_segment(const location& location) {
  auto it = std::lower_bound(
      state_.slocations.begin(), state_.slocations.end(), slocation{},
      [&location, this](const slocation& lhs, const slocation& rhs) {
        return location_less(lhs.location(state_.segment_list, location),
                             rhs.location(state_.segment_list, location));
      });

  utl::verify(it != state_.slocations.end(),
              "get_next_segment: no slocation at requested location");
  if (state_.segment_list[it->item].is_done()) {
    ++it;
  }
  utl::verify(it != state_.slocations.end(),
              "get_next_segment: ran past end looking for an open segment");
  utl::verify(!state_.segment_list[it->item].is_done(),
              "get_next_segment: candidate segment is already done");
  return &state_.segment_list[it->item];
}

void remove_duplicates(std::vector<rings_stack_element>& outer_rings) {
  while (true) {
    auto const it = std::adjacent_find(outer_rings.begin(), outer_rings.end());
    if (it == outer_rings.end()) {
      return;
    }
    outer_rings.erase(it, std::next(it, 2));
  }
}

std::vector<location_to_ring_map> assembly::create_location_to_ring_map(
    open_ring_its_type& open_ring_its) {
  auto xrings = std::vector<location_to_ring_map>{};
  xrings.reserve(open_ring_its.size() * 2);

  auto count_for_debug = 0;
  for (auto it = open_ring_its.begin(); it != open_ring_its.end(); ++it) {
    if (state_.debug) {
      ++count_for_debug;
      dbg("      {}", count_for_debug);
    }
    xrings.emplace_back((*it)->get_node_ref_start().location(), it, true);
    xrings.emplace_back((*it)->get_node_ref_stop().location(), it, false);
  }
  std::stable_sort(xrings.begin(), xrings.end());
  return xrings;
}

void assembly::merge_two_rings(open_ring_its_type& open_ring_its,
                               const location_to_ring_map& m1,
                               const location_to_ring_map& m2) {
  auto const r1 = *m1.ring_it;
  auto const r2 = *m2.ring_it;

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
    // Unreachable: m1 and m2 were paired by a shared location, so it is an
    // endpoint of both rings -> one of the four cases above always matches.
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

bool assembly::try_to_merge(open_ring_its_type& open_ring_its) {
  if (open_ring_its.empty()) {
    return false;
  }
  dbg("    Trying to merge {} open rings (try_to_merge)",
      open_ring_its.size());

  auto const xrings = create_location_to_ring_map(open_ring_its);
  auto it = xrings.cbegin();
  while (it != xrings.cend()) {
    it = std::adjacent_find(it, xrings.cend());
    if (it == xrings.cend()) {
      return false;
    }
    auto const after = std::next(it, 2);
    if (after == xrings.cend() || after->location != it->location) {
      dbg("      Merging two rings");
      merge_two_rings(open_ring_its, *it, *std::next(it));
      return true;
    }
    while (it != xrings.cend() && it->location == after->location) {
      ++it;
    }
  }
  return false;
}

proto_ring* assembly::find_enclosing_ring(node_ref_segment* segment) {
  dbg("    Looking for ring enclosing ");

  auto const location = segment->first().location();
  auto const end_location = segment->second().location();

  while (segment->first().location() == location) {
    if (segment == &state_.segment_list.back()) {
      break;
    }
    ++segment;
  }

  auto nesting = 0;
  auto outer_rings = std::vector<rings_stack_element>{};
  while (segment >= &state_.segment_list.front()) {
    if (!segment->is_direction_done()) {
      --segment;
      continue;
    }
    dbg("      Checking against {}", fmt::ptr(segment));
    auto const& a = segment->first().location();
    auto const& b = segment->second().location();

    if (segment->first().location() == location) {
      auto const ax = static_cast<std::int64_t>(a.x());
      auto const bx = static_cast<std::int64_t>(b.x());
      auto const lx = static_cast<std::int64_t>(end_location.x());
      auto const ay = static_cast<std::int64_t>(a.y());
      auto const by = static_cast<std::int64_t>(b.y());
      auto const ly = static_cast<std::int64_t>(end_location.y());
      auto const z = ((bx - ax) * (ly - ay)) - ((by - ay) * (lx - ax));
      dbg("      Segment z={}", z);
      if (z > 0) {
        nesting += segment->is_reverse() ? -1 : 1;
        dbg("        Segment is below (nesting={})", nesting);
        if (segment->ring()->is_outer()) {
          dbg("        Segment belongs to outer ring (y={} ring={})", a.y(),
              fmt::ptr(segment->ring()));
          outer_rings.emplace_back(a.y(), segment->ring());
        }
      }
    } else if (a.x() <= location.x() && location.x() < b.x()) {
      dbg("        Is in x range");

      auto const ax = static_cast<std::int64_t>(a.x());
      auto const bx = static_cast<std::int64_t>(b.x());
      auto const lx = static_cast<std::int64_t>(location.x());
      auto const ay = static_cast<std::int64_t>(a.y());
      auto const by = static_cast<std::int64_t>(b.y());
      auto const ly = static_cast<std::int64_t>(location.y());
      auto const z = ((bx - ax) * (ly - ay)) - ((by - ay) * (lx - ax));

      if (z >= 0) {
        nesting += segment->is_reverse() ? -1 : 1;
        dbg("        Segment is below (nesting={})", nesting);
        if (segment->ring()->is_outer()) {
          auto const y = static_cast<double>(ay) +
                         (static_cast<double>((by - ay) * (lx - ax)) /
                          static_cast<double>(bx - ax));
          dbg("        Segment belongs to outer ring (y={} ring={})", y,
              fmt::ptr(segment->ring()));
          outer_rings.emplace_back(y, segment->ring());
        }
      }
    }
    --segment;
  }

  if (nesting % 2 == 0) {
    dbg("    Decided that this is an outer ring");
    return nullptr;
  }
  dbg("    Decided that this is an inner ring");
  std::stable_sort(outer_rings.rbegin(), outer_rings.rend());
  if (state_.debug) {
    for (auto const& o : outer_rings) {
      dbg("        y={}", o.y());  // " " << o.ring() << "\n";
    }
  }
  remove_duplicates(outer_rings);
  if (state_.debug) {
    dbg("      after remove duplicates:");
    for (auto const& o : outer_rings) {
      dbg("        y={}", o.y());  //" " << o.ring() << "\n";
    }
  }

  if (outer_rings.empty()) {
    // Parity counted this as an inner ring, but no enclosing outer ring
    // survived (all candidates belonged to inner rings, or cancelled out in
    // remove_duplicates). Fall back to treating it as an outer ring rather
    // than dereferencing an empty vector (the `assert` above is a no-op under
    // NDEBUG). Prevents a crash on lakes-with-islands and similar geometries.
    dbg("    No enclosing outer ring found -> treating as outer ring");
    return nullptr;
  }
  return outer_rings.front().ring_ptr();
}

void assembly::find_inner_outer_complex(proto_ring* ring) {
  auto* outer_ring = find_enclosing_ring(ring->min_segment());
  if (outer_ring) {
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  }
  ring->fix_direction();
  ring->mark_direction_done();
}

void assembly::find_inner_outer_complex() {
  dbg("  Finding inner/outer rings");
  auto closed_rings = std::vector<proto_ring*>{};
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
                   [](proto_ring* a, proto_ring* b) {
                     return a->min_segment() < b->min_segment();
                   });
  closed_rings.front()->fix_direction();
  closed_rings.front()->mark_direction_done();
  dbg("    First ring is outer: {}", fmt::ptr(closed_rings.front()));
  auto count_for_debug = 0;
  for (auto it = std::next(closed_rings.begin()); it != closed_rings.end();
       ++it) {
    ++count_for_debug;
    dbg("    Checking (at min segment {}) ring {}",
        fmt::ptr((*it)->min_segment()), count_for_debug);  //**it << "\n";
    find_inner_outer_complex(*it);
    dbg("    Ring is {}{}", (*it)->is_outer() ? "OUTER: " : "INNER: ",
        count_for_debug);  //**it << "\n";
  }
}

static void find_candidates(std::vector<candidate>& candidates,
                            std::vector<location>& loc_done,
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

  auto const connections =
      make_range(std::equal_range(xrings.cbegin(), xrings.cend(),
                                  location_to_ring_map{cand.stop_location}));

  assert(connections.begin() != connections.end());

  utl::verify(!cand.rings.empty(), "find_candidates: candidate has no rings");
  auto const* ring_leading_here = &cand.rings.back().first.ring();
  for (auto const& m : connections) {
    auto const& ring = m.ring();

    if (&ring != ring_leading_here) {
      // if (debug) {
      //   std::cerr << "        next possible connection: ";
      //   ring.print(std::cerr);
      //   std::cerr << (m.start ? "" : " reverse") << "\n";
      // }

      auto c = cand;
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
        utl::verify(!loc_done.empty() && loc_done.back() == c.stop_location,
                    "find_candidates: loc_done stack corrupted after recursion");
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
bool assembly::join_connected_rings(open_ring_its_type& open_ring_its) {
  // Caller (create_rings_complex_case) only calls this when non-empty.
  assert(!open_ring_its.empty());
  dbg("    Trying to merge {} open rings (join_connected_rings)",
      open_ring_its.size());
  auto xrings = create_location_to_ring_map(open_ring_its);

  auto const ring_min = std::min_element(
      xrings.begin(), xrings.end(),
      [](const location_to_ring_map& lhs, const location_to_ring_map& rhs) {
        return lhs.ring().min_segment() < rhs.ring().min_segment();
      });

  find_inner_outer_complex();
  auto const* outer_ring = find_enclosing_ring(ring_min->ring().min_segment());
  auto const ring_min_is_outer = !outer_ring;
  dbg("  Open ring is {} ring", ring_min_is_outer ? "outer" : "inner");
  for (auto& ring : state_.rings) {
    ring.reset();
  }

  auto const cand = candidate{*ring_min, false};
  auto loc_done = std::vector<location>{};
  loc_done.push_back(cand.stop_location);
  auto candidates = std::vector<candidate>{};

  try {
    find_candidates(candidates, loc_done, xrings, cand, state_.debug);
  } catch (std::exception e) {
    dbg("Exceeded max depth ({})", max_depth);
    return false;
  }

  if (candidates.empty()) {
    dbg("    Found no candidates");
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
    dbg("    Found candidates:");
    for (auto const& c : candidates) {
      dbg("      sum={}", c.sum);
      // for (const auto& ring : c.rings) {
      //   std::cerr << "        ";
      //   ring.first.ring().print(std::cerr);
      //   std::cerr << (ring.second ? " reverse" : "") << "\n";
      // }
    }
  }

  // Find the candidate with the smallest/largest area
  auto const chosen_cand =
      ring_min_is_outer ? candidates.front() : candidates.back();

  dbg("    Decided on: sum={}", chosen_cand.sum);
  // for (const auto& ring : chosen_cand.rings) {
  //   std::cerr << "        ";
  //   ring.first.ring().print(std::cerr);
  //   std::cerr << (ring.second ? " reverse" : "") << "\n";
  // }

  // Join all (open) rings in the candidate to get one closed ring.
  assert(chosen_cand.rings.size() > 1);
  auto const& first_ring = chosen_cand.rings.front().first;
  auto const& remaining_ring = first_ring.ring();
  for (auto it = std::next(chosen_cand.rings.begin());
       it != chosen_cand.rings.end(); ++it) {
    merge_two_rings(open_ring_its, first_ring, it->first);
  }

  dbg("    Merged ");  // to ";
  // remaining_ring.print(std::cerr);
  // std::cerr << '\n';
  return true;
}

std::uint32_t assembly::add_new_ring_complex(const slocation& node) {
  auto* segment = &state_.segment_list[node.item];
  assert(!segment->is_done());  // caller only starts a ring on an open segment
  dbg("  Starting new ring at location {},{} with segment {}",
      node.location(state_.segment_list).x(),
      node.location(state_.segment_list).y(), fmt::ptr(segment));
  if (node.reverse) {
    segment->reverse();
  }
  state_.rings.emplace_back(segment);
  auto* ring = &state_.rings.back();

  auto const& first_location = node.location(state_.segment_list);
  auto last_location = segment->stop().location();

  auto is_split_location = [&](location loc) {
    return std::find(state_.split_locations.cbegin(),
                     state_.split_locations.cend(),
                     loc) != state_.split_locations.cend();
  };

  auto nodes = std::uint32_t{1};
  while (first_location != last_location && !is_split_location(last_location)) {
    ++nodes;
    auto* next_segment = assembly::get_next_segment(last_location);
    if (next_segment->start().location() != last_location) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    dbg("    Next segment is {}", fmt::ptr(next_segment));
    last_location = next_segment->stop().location();
  }
  if (first_location == last_location) {
    dbg("    Completed ring: {}", fmt::ptr(ring));
  } else {
    dbg("    Completed partial ring: {}", fmt::ptr(ring));
  }
  return nodes;
}

std::uint32_t assembly::add_new_ring(const slocation& node) {
  auto* segment = &state_.segment_list[node.item];
  assert(!segment->is_done());  // caller only starts a ring on an open segment
  dbg("  Starting new ring at location {},{} with segment {}",
      node.location(state_.segment_list).x(),
      node.location(state_.segment_list).y(), fmt::ptr(segment));
  if (node.reverse) {
    segment->reverse();
  }

  proto_ring* outer_ring = nullptr;
  if (segment != &state_.segment_list.front()) {
    outer_ring = find_enclosing_ring(segment);
  }
  segment->mark_direction_done();
  state_.rings.emplace_back(segment);
  auto* ring = &state_.rings.back();
  if (outer_ring) {
    dbg("    This is an inner ring:");
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  } else {
    dbg("    This is an outer ring");
  }

  auto const& first_location = node.location(state_.segment_list);
  auto last_location = segment->stop().location();

  auto nodes = std::uint32_t{1};
  while (first_location != last_location) {
    ++nodes;
    auto* next_segment = get_next_segment(last_location);
    next_segment->mark_direction_done();
    if (next_segment->start().location() != last_location) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    dbg("    Next segment is {}", next_segment->first_noderef_.ref());
    last_location = next_segment->stop().location();
  }

  ring->fix_direction();

  dbg("    Completed ring ");
  return nodes;
}

bool assembly::create_rings_complex_case() {
  // First create all the (partial) rings starting at the split locations
  auto count_remaining = state_.segment_list.size();
  for (auto const& location : state_.split_locations) {
    auto const locs = make_range(std::equal_range(
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
    for (auto const& sl : state_.slocations) {
      auto const& segment = state_.segment_list[sl.item];
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
                       [](const proto_ring& ring) { return !ring.closed(); });
  };

  // Now all segments are in exactly one (partial) ring.
  // If there are open rings, try to join them to create closed
  // rings.
  if (there_are_open_rings()) {
    ++state_.stats.area_really_complex_case;

    auto open_ring_its = open_ring_its_type{};
    for (auto it = state_.rings.begin(); it != state_.rings.end(); ++it) {
      if (!it->closed()) {
        open_ring_its.push_back(it);
      }
    }

    while (!open_ring_its.empty()) {
      dbg("There are {} open rings", open_ring_its.size());
      while (try_to_merge(open_ring_its)) {
        // intentionally left blank
      }
      if (!open_ring_its.empty()) {
        dbg("  After joining obvious cases there are still {} open rings",
            open_ring_its.size());
        if (!join_connected_rings(open_ring_its)) {
          return false;
        }
      }
    }

    dbg("  Joined all open rings");
  }

  // Now all rings are complete.
  find_inner_outer_complex();

  return true;
}

bool assembly::create_rings() {
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
    dbg("  No segments left");
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
  assert(state_.segment_list.size() <
         std::numeric_limits<std::uint32_t>::max());
  for (std::uint32_t n = 0;
       n < static_cast<std::uint32_t>(state_.segment_list.size()); ++n) {
    state_.slocations.emplace_back(n, false);
    state_.slocations.emplace_back(n, true);
  }
  std::stable_sort(state_.slocations.begin(), state_.slocations.end(),
                   [this](slocation const& lhs, slocation const& rhs) {
                     return location_less(lhs.location(state_.segment_list),
                                          rhs.location(state_.segment_list));
                   });
  // Find all locations where more than two segments start or
  // end. We call those "split" locations. If there are any
  // "spike" segments found while doing this, we know the area
  // geometry isn't valid and return.
  /**
   * If there are any open rings found along the way, they are reported
   * and the function returns false.
   */
  auto found_open_rings = false;
  auto previous_location = location{};
  for (auto it = state_.slocations.cbegin(); it != state_.slocations.cend();
       ++it) {
    auto const& nr = it->node_ref(state_.segment_list);
    auto const& loc = nr.location();
    if (std::next(it) == state_.slocations.cend() ||
        loc != std::next(it)->location(state_.segment_list)) {
      dbg(" Found open ring at {}", nr.ref());
      auto const& segment = state_.segment_list[it->item];
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
    dbg("  Found split locations:");
    for (auto const& location : state_.split_locations) {
      auto const it = std::lower_bound(
          state_.slocations.cbegin(), state_.slocations.cend(), slocation{},
          [this, &location](slocation const& lhs, slocation const& rhs) {
            return lhs.location(state_.segment_list, location) <
                   rhs.location(state_.segment_list, location);
          });
      assert(it != state_.slocations.cend());
      auto const id = it->node_ref(state_.segment_list).ref();
      state_.problem_reporter.report_touching_ring(id, location);
      dbg("    {},{}", location.x(), location.y());
    }
  }
  // From here on we use two different algorithms depending on
  // whether there were any split locations or not. If there
  // are no splits, we use the faster "simple algorithm", if
  // there are, we use the slower "complex algorithm".
  if (state_.split_locations.empty()) {
    dbg(" No split locations -> using simple algorithm");
    ++state_.stats.area_simple_case;
    // create_rings_simple_case:
    auto count_remaining = state_.segment_list.size();
    for (auto const& sl : state_.slocations) {
      auto const& segment = state_.segment_list[sl.item];
      if (!segment.is_done()) {
        count_remaining -= add_new_ring(sl);
        if (count_remaining == 0) {
          break;
        }
      }
    }  // create_rings_simple_case - finished
  } else if (state_.split_locations.size() > max_split_locations_) {
    dbg(" Ignoring polygon with {} split locations (>{})",
        state_.split_locations.size(), max_split_locations_);
    return false;
  } else {
    dbg(" Found {} split locations -> using complex algorithm",
        state_.split_locations.size());
    ++state_.stats.area_touching_rings_case;
    if (!create_rings_complex_case()) {
      return false;
    }
  }
  // If the assembler was so configured, now check whether the
  // member roles are correctly tagged. --> check always
  // check_inner_outer_roles:
  dbg("    Checking inner/outer roles");
  auto count_segments_for_debug = 0;
  auto way_rings = std::unordered_map<way const*, proto_ring const*>{};
  auto ways_in_multiple_rings = std::unordered_set<way const*>{};
  for (auto const& ring : state_.rings) {
    for (auto const& segment : ring.segments()) {
      count_segments_for_debug++;
      assert(segment->way());
      if (!segment->role_empty() &&
          (ring.is_outer() ? !segment->role_outer()
                           : !segment->role_inner())) {
        ++state_.stats.wrong_role;
        dbg(" Segment: {} from way {} has role '{}', but should have role "
            "'{}'\n ",
            count_segments_for_debug, segment->way()->id,
            segment->role_name(), ring.is_outer() ? "outer" : "inner");
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
  for (auto const* way :
       ways_in_multiple_rings) {  // NOLINT(bugprone - nondeterministic -
                                  // pointer - iteration - order)
    ++state_.stats.ways_in_multiple_rings;
    dbg(" way {} is in multiple rings\n ", way->id);
    state_.problem_reporter.report_way_in_multiple_rings(*way);
  }
  // check_inner_outer_roles - finished

  state_.stats.outer_rings =
      std::count_if(state_.rings.cbegin(), state_.rings.cend(),
                    [](proto_ring const& ring) { return ring.is_outer(); });
  state_.stats.inner_rings = state_.rings.size() - state_.stats.outer_rings;
  return true;
}

bool assembly::create_area(polygon_area& out_buffer) {
  auto const area_okay = create_rings();
  if (area_okay) {
    auto area = std::vector<area_pair>{};
    for (auto const& ring : state_.rings) {
      if (!ring.is_outer()) {
        continue;
      }
      auto area_part = std::vector<node_ref>{};
      auto offsets = std::vector<std::int64_t>{};
      offsets.push_back(area_part.size());
      area_part.emplace_back(ring.get_node_ref_start());
      for (auto const& segment : ring.segments()) {
        area_part.emplace_back(segment->stop());
      }
      for (auto const* inner : ring.inner_rings()) {
        offsets.push_back(area_part.size());
        area_part.emplace_back(inner->get_node_ref_start());
        for (auto const& segment : inner->segments()) {
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

bool assembly::assembling_area_from_way(way const& way,
                                        polygon_area& out_buffer) {
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
  dbg("\nAssembling way {} containing {} nodes", way.id,
      state_.segment_list.size());
  auto const okay = create_area(out_buffer);
  dbg("Done: {}", way.id);
  if (state_.debug) {
    state_.stats.print_stats();
  }
  return okay;
}

}  // namespace osm
