#pragma once

#include <cstdint>
#include <vector>

#include "geo/fixed_latlng.h"

#include "osm/varint.h"

namespace osm {

enum class member_type : std::uint32_t { kNode, kWay, kRelation };
using object_id_type = std::int64_t;
using location = geo::fixed_latlng;

struct node {
  // Use the qualified geo type inside the struct to avoid colliding with the
  // `location()` method name in class scope.
  geo::fixed_latlng location() const noexcept { return node_loc_; }
  object_id_type id() const noexcept { return id_; }
  node& set_location(geo::fixed_latlng const& l) noexcept {
    node_loc_ = l;
    return *this;
  }

  object_id_type id_;
  geo::fixed_latlng node_loc_;
};

struct node_ref {
  constexpr object_id_type ref() const { return ref_id; }
  geo::fixed_latlng const& location() const noexcept { return loc; }
  geo::fixed_latlng& location() noexcept { return loc; }
  void set_location(geo::fixed_latlng l) { loc = l; }

  object_id_type ref_id;
  geo::fixed_latlng loc;
};

struct way {
  std::vector<node_ref>& nodes() { return node_refs; }
  std::vector<node_ref> const& nodes() const { return node_refs; }
  bool ends_have_same_id() const noexcept {
    return nodes().front().ref() == nodes().back().ref();
  }

  object_id_type id;
  std::vector<node_ref> node_refs;
};

template <typename Members>
struct relation {
  constexpr Members members() const { return members_; }

  object_id_type id;
  Members members_;
};

}  // namespace osm
