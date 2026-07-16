#pragma once

#include <cstdint>
#include <span>

#include "osmium/builder/osm_object_builder.hpp"
#include "osmium/memory/buffer.hpp"
#include "osmium/osm/area.hpp"
#include "osmium/osm/location.hpp"
#include "osmium/osm/node.hpp"
#include "osmium/osm/node_ref.hpp"
#include "osmium/osm/relation.hpp"
#include "osmium/osm/way.hpp"

#include "geo/latlng.h"

#include "osm/assembler/assembler_types.h"
#include "osm/types.h"

// Builders that materialize osmium objects from decoded osm:: primitives in a
// caller-provided buffer. They let downstream code that consumes
// osmium::Node/Way/Area/Relation (tag parsers, handlers, ...) stay unchanged
// when the input is read with osm::raw_reader/decode_primitive instead of
// libosmium I/O. The returned reference points into the buffer and is valid
// until the next build_* call on the same buffer.

namespace osm {

// `osmium::Location` is constructed `(x = lng, y = lat)` in the same
// fixed-precision units as `geo::fixed_latlng`'s `lng_`/`lat_` members.
inline osmium::Location to_osmium(osm::location const& l) noexcept {
  return osmium::Location{l.lng_, l.lat_};
}

template <typename Tags>
void add_tags(osmium::builder::Builder& parent, Tags&& tags) {
  osmium::builder::TagListBuilder tl{parent};
  for (auto const& [k, v] : tags) {
    tl.add_tag(k.data(), k.size(), v.data(), v.size());
  }
}

template <typename Tags>
osmium::Node const& build_node(osmium::memory::Buffer& buf,
                               std::int64_t const id,
                               geo::latlng const& pos,
                               Tags&& tags) {
  buf.clear();
  {
    osmium::builder::NodeBuilder nb{buf};
    nb.set_id(id);
    nb.set_location(osmium::Location{pos.lng_, pos.lat_});
    add_tags(nb, std::forward<Tags>(tags));
  }
  buf.commit();
  return buf.get<osmium::Node>(0);
}

template <typename Tags>
osmium::Way const& build_way(osmium::memory::Buffer& buf,
                             osm::way const& w,
                             Tags&& tags) {
  buf.clear();
  {
    osmium::builder::WayBuilder wb{buf};
    wb.set_id(w.id);
    {
      osmium::builder::WayNodeListBuilder wnl{wb};
      for (auto const& nr : w.nodes()) {
        wnl.add_node_ref(osmium::NodeRef{nr.ref(), to_osmium(nr.location())});
      }
    }
    add_tags(wb, std::forward<Tags>(tags));
  }
  buf.commit();
  return buf.get<osmium::Way>(0);
}

inline osmium::item_type to_osmium(member_type const t) noexcept {
  switch (t) {
    case member_type::kNode: return osmium::item_type::node;
    case member_type::kWay: return osmium::item_type::way;
    case member_type::kRelation: return osmium::item_type::relation;
  }
  return osmium::item_type::undefined;
}

// Members: range of tuple-likes [ref (std::int64_t), role (string_view),
// type (osm::member_type)] as produced by decode_primitive.
template <typename Members, typename Tags>
osmium::Relation const& build_relation(osmium::memory::Buffer& buf,
                                       std::int64_t const id,
                                       Members&& members,
                                       Tags&& tags) {
  buf.clear();
  {
    osmium::builder::RelationBuilder rb{buf};
    rb.set_id(id);
    {
      osmium::builder::RelationMemberListBuilder ml{rb};
      for (auto const& [ref, role, type] : members) {
        ml.add_member(to_osmium(type), ref, role.data(), role.size());
      }
    }
    add_tags(rb, std::forward<Tags>(tags));
  }
  buf.commit();
  return buf.get<osmium::Relation>(0);
}

namespace detail {

template <typename RingBuilder>
void emit_ring(osmium::builder::AreaBuilder& ab,
               std::span<osm::node_ref const> const ring) {
  RingBuilder rb{ab};
  for (auto const& nr : ring) {
    rb.add_node_ref(osmium::NodeRef{nr.ref(), to_osmium(nr.location())});
  }
}

}  // namespace detail

template <typename Tags>
osmium::Area const& build_area(osmium::memory::Buffer& buf,
                               osm::polygon_area const& pa,
                               Tags&& tags) {
  buf.clear();
  {
    osmium::builder::AreaBuilder ab{buf};
    // Use the original (way/relation) id directly; downstream handlers do
    // not depend on osmium's area-id scheme.
    ab.set_id(pa.origin_id);
    add_tags(ab, std::forward<Tags>(tags));

    for (auto const& ap : pa.area) {
      if (ap.offsets.empty()) {
        continue;
      }
      auto const part_size = static_cast<std::int64_t>(ap.area_part.size());
      // outer ring
      auto const outer_start = ap.offsets[0];
      auto const outer_end = ap.offsets.size() > 1 ? ap.offsets[1] : part_size;
      if (outer_end > outer_start) {
        detail::emit_ring<osmium::builder::OuterRingBuilder>(
            ab, std::span<osm::node_ref const>{
                    ap.area_part.data() + outer_start,
                    static_cast<std::size_t>(outer_end - outer_start)});
      }
      // inner rings
      for (std::size_t i = 1; i < ap.offsets.size(); ++i) {
        auto const inner_start = ap.offsets[i];
        auto const inner_end =
            (i + 1 < ap.offsets.size()) ? ap.offsets[i + 1] : part_size;
        if (inner_end > inner_start) {
          detail::emit_ring<osmium::builder::InnerRingBuilder>(
              ab, std::span<osm::node_ref const>{
                      ap.area_part.data() + inner_start,
                      static_cast<std::size_t>(inner_end - inner_start)});
        }
      }
    }
  }
  buf.commit();
  return buf.get<osmium::Area>(0);
}

}  // namespace osm
