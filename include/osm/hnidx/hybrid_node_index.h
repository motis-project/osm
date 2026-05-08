#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cista/mmap.h"

#include "geo/fixed_latlng.h"

#include "osm/hnidx/fixed_geometry.h"
#include "osm/types.h"

namespace osm {

struct hybrid_node_idx {
  static constexpr auto x_offset =
      180 * geo::fixed_latlng::kCoordinatePrecision;
  static constexpr auto y_offset = 90 * geo::fixed_latlng::kCoordinatePrecision;

  hybrid_node_idx(cista::mmap idx_mmap, cista::mmap dat_mmap);
  ~hybrid_node_idx();

  hybrid_node_idx(hybrid_node_idx const&) = delete;
  hybrid_node_idx(hybrid_node_idx&&) noexcept = default;
  hybrid_node_idx& operator=(hybrid_node_idx const&) = delete;
  hybrid_node_idx& operator=(hybrid_node_idx&&) noexcept = default;

  struct impl;
  std::unique_ptr<impl> impl_;
};

std::optional<fixed_xy> get_coords(hybrid_node_idx const&,
                                   osm::object_id_type id);

void get_coords(
    hybrid_node_idx const&,
    std::vector<std::pair<osm::object_id_type, osm::location*>>& queries);

void update_locations_of_way(hybrid_node_idx const&, osm::way&);

void prefetch_way_locations(hybrid_node_idx const&, osm::way const&);

// Encoded representation of a single PBF block's node positions. Each
// block is self-contained: every coord-span starts with absolute (fixed)
// coords, so encoding can be done in parallel across worker fibers.
// `finish()` terminates the byte stream with an end-of-block marker (a
// zero-size empty-span varint = byte 0x01) so the read path knows where
// the block ends without an explicit length field.
struct hybrid_block {
  struct span_start {
    osm::object_id_type start_id_;
    std::size_t offset_in_encoded_;
    std::size_t coords_before_;  // coords in this block's earlier spans
  };

  osm::object_id_type first_id_{
      std::numeric_limits<osm::object_id_type>::min()};
  osm::object_id_type last_id_{std::numeric_limits<osm::object_id_type>::min()};
  std::size_t total_coords_{0U};
  std::string encoded_;
  std::vector<span_start> span_starts_;
};

struct hybrid_block_encoder {
  hybrid_block_encoder() = default;

  hybrid_block_encoder(hybrid_block_encoder const&) = delete;
  hybrid_block_encoder(hybrid_block_encoder&&) noexcept = default;
  hybrid_block_encoder& operator=(hybrid_block_encoder const&) = delete;
  hybrid_block_encoder& operator=(hybrid_block_encoder&&) noexcept = default;

  void node(osm::node const& n) {
    push(n.id(), fixed_xy{static_cast<fixed_coord_t>(n.location().x()) +
                              hybrid_node_idx::x_offset,
                          static_cast<fixed_coord_t>(n.location().y()) +
                              hybrid_node_idx::y_offset});
  }

  void push(osm::object_id_type id, fixed_xy const& pos);

  hybrid_block finish() &&;

  hybrid_block result_;
  std::vector<fixed_xy> span_;
  osm::object_id_type last_id_{std::numeric_limits<osm::object_id_type>::min()};
  fixed_xy last_pos_{0, 0};
};

// Single-fiber merger that consumes `hybrid_block`s in input (file) order
// and writes the global `idx_`/`dat_`. Per-block work is one bulk
// `memcpy` of the encoded byte stream into `dat_` plus a few
// `idx_.push_back`s.
struct hybrid_block_merger {
  explicit hybrid_block_merger(hybrid_node_idx&);
  ~hybrid_block_merger();

  hybrid_block_merger(hybrid_block_merger const&) = delete;
  hybrid_block_merger(hybrid_block_merger&&) noexcept = default;
  hybrid_block_merger& operator=(hybrid_block_merger const&) = delete;
  hybrid_block_merger& operator=(hybrid_block_merger&&) noexcept = default;

  void merge(hybrid_block&& block);
  void finish();

  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osm
