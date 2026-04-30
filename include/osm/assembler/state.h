#pragma once

#include <list>

#include "osm/assembler/assembler_stats.h"
#include "osm/assembler/assembler_types.h"
#include "osm/assembler/problem_reporter.h"

namespace osm {

constexpr std::size_t max_split_locations_ = 100ULL;
constexpr int max_depth = 20;

struct state {
  state() = delete;
  state(const state&) = delete;
  state& operator=(const state&) = delete;
  explicit state(std::ostream* out, bool enable_debug = true)
      : problem_reporter{out, enable_debug}, debug{enable_debug} {}

  bool debug;
  segment_list segment_list{};
  area_stats stats{};
  problem_reporter problem_reporter;
  std::list<proto_ring> rings{};
  std::vector<slocation> slocations{};
  std::vector<location> split_locations{};
};

}  // namespace osm