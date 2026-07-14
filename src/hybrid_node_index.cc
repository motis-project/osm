#include "osm/hnidx/hybrid_node_index.h"

#include <sys/mman.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <string>
#include <tuple>

#include "cista/containers/mmap_vec.h"

#include "protozero/varint.hpp"
#include "utl/verify.h"

#include "osm/hnidx/delta.h"

namespace pz = protozero;

namespace osm {

// On-disk format:
//
// `dat_` is a sequence of blocks concatenated back-to-back. Each block is
// a self-contained varint-prefixed span stream, terminated by a zero-size
// empty-span varint (byte 0x01) that bounds the read-path walker:
//   header := varint( (count << 1) | kind )
//     kind = 0 → coord-span: `count + 1` consecutive node ids
//                 [u32 fixed x0][u32 fixed y0]
//                 [zigzag-varint dx1][zigzag-varint dy1] ...
//     kind = 1, count > 0 → empty-span: `count` missing ids since the
//                previous span's last id (in-block gaps).
//     kind = 1, count = 0 → end-of-block (terminates the walk).
//
// `idx_` is sorted `id_offset{ start_id, block_offset, in_block_offset }`,
// with a mandatory entry at every block boundary plus sparse entries
// inside each block (~one per `kCoordsPerIndex` written coords). Lookup
// binary-searches `idx_`, then walks forward from
// `dat_.data() + block_offset + in_block_offset` until the EOB marker.
// Walks can never escape the block: queries that fall in inter-block id
// gaps are detected by the batch FSM (block already exhausted, lookup
// lands back in it) and skipped without re-walking.

struct id_offset {
  id_offset() = default;
  id_offset(osm::object_id_type const id, std::uint64_t const block_offset,
            std::uint32_t const in_block_offset)
      : id_{id},
        block_offset_{block_offset},
        in_block_offset_{in_block_offset} {}

  bool operator==(id_offset const&) const = default;

  osm::object_id_type id_{std::numeric_limits<osm::object_id_type>::max()};
  std::uint64_t block_offset_{std::numeric_limits<std::uint64_t>::max()};
  std::uint32_t in_block_offset_{0U};
};

struct hybrid_node_idx::impl {
  impl(cista::mmap idx_mmap, cista::mmap dat_mmap)
      : idx_{std::move(idx_mmap)}, dat_{std::move(dat_mmap)} {}

  cista::basic_mmap_vec<id_offset, std::uint64_t> idx_;
  cista::basic_mmap_vec<char, std::uint64_t> dat_;
};

namespace {

constexpr auto kCoordsPerIndex = std::size_t{1024U};

inline std::uint32_t read_fixed(char const** data) {
  auto const* p = reinterpret_cast<std::uint8_t const*>(*data);
  std::uint32_t val = 0;
  val |= *p++;
  val |= static_cast<std::uint32_t>(*p++) << 8U;
  val |= static_cast<std::uint32_t>(*p++) << 16U;
  val |= static_cast<std::uint32_t>(*p++) << 24U;
  *data = reinterpret_cast<char const*>(p);
  return val;
}

inline std::uint32_t get_varint_size(std::uint64_t value) {
  auto n = std::uint32_t{1U};
  while (value >= 0x80ULL) {
    value >>= 7U;
    ++n;
  }
  return n;
}

}  // namespace

// ----- read path -------------------------------------------------------

std::optional<fixed_xy> get_coords(hybrid_node_idx const& nodes,
                                   osm::object_id_type const id) {
  auto const& idx = nodes.impl_->idx_;
  auto const& dat = nodes.impl_->dat_;

  if (idx.empty()) {
    return std::nullopt;
  }
  auto const abs_id = std::abs(id);
  auto const* it =
      std::lower_bound(std::begin(idx), std::end(idx), abs_id,
                       [](auto const& o, auto const& i) { return o.id_ < i; });

  if (it == std::begin(idx) && it->id_ != abs_id) {
    return std::nullopt;
  }
  if (it == std::end(idx) || it->id_ != abs_id) {
    --it;
  }

  auto curr_id = it->id_;
  auto const* dat_it = reinterpret_cast<char const*>(dat.data()) +
                       it->block_offset_ + it->in_block_offset_;
  auto const* const dat_global_end =
      reinterpret_cast<char const*>(dat.data()) + dat.size();

  while (curr_id <= abs_id && dat_it != dat_global_end) {
    auto const header = pz::decode_varint(&dat_it, dat_global_end);
    auto const span_size = header >> 1U;
    auto const kind = header & 0x1U;

    if (kind == 0x1U) {
      if (span_size == 0U) {
        break;  // end of this block
      }
      curr_id += static_cast<osm::object_id_type>(span_size);
      continue;
    }

    delta_decoder x_dec{static_cast<fixed_coord_t>(read_fixed(&dat_it))};
    delta_decoder y_dec{static_cast<fixed_coord_t>(read_fixed(&dat_it))};
    if (curr_id == abs_id) {
      return fixed_xy{x_dec.curr_, y_dec.curr_};
    }
    ++curr_id;

    for (auto i = std::uint64_t{1U}; i < span_size + 1U; ++i) {
      auto const x = x_dec.decode(static_cast<fixed_delta_t>(
          pz::decode_zigzag64(pz::decode_varint(&dat_it, dat_global_end))));
      auto const y = y_dec.decode(static_cast<fixed_delta_t>(
          pz::decode_zigzag64(pz::decode_varint(&dat_it, dat_global_end))));
      if (curr_id == abs_id) {
        return fixed_xy{x, y};
      }
      ++curr_id;
    }
  }
  return std::nullopt;
}

void get_coords(
    hybrid_node_idx const& nodes,
    std::vector<std::pair<osm::object_id_type, osm::location*>>& queries) {
  auto const& idx = nodes.impl_->idx_;
  auto const& dat = nodes.impl_->dat_;

  if (idx.empty()) {
    return;
  }

  auto const* const dat_base = reinterpret_cast<char const*>(dat.data());
  auto const* const dat_global_end = dat_base + dat.size();

  auto curr_id =
      osm::object_id_type{std::numeric_limits<osm::object_id_type>::min()};
  char const* dat_it = nullptr;
  auto span_size = std::int64_t{0};
  auto span_pos = std::int64_t{0};
  // Block we last entered via `from_index`; when an EOB ends the walk and
  // the next binary-search lands back in this block, the query is in an
  // inter-block id gap and must be skipped to make progress.
  auto last_block_offset = std::numeric_limits<std::uint64_t>::max();
  auto block_exhausted = false;

  delta_decoder x_dec{0};
  delta_decoder y_dec{0};

  enum class fsm_state {
    from_index,
    at_span_start,
    in_span,
  };

  auto state = fsm_state::from_index;

  std::sort(begin(queries), end(queries), [](auto const& a, auto const& b) {
    return std::abs(a.first) < std::abs(b.first);
  });

  auto q_it = begin(queries);
  for (; q_it != end(queries) && std::abs(q_it->first) < idx[0].id_; ++q_it) {
    // skip queries below the indexed range
  }

  constexpr auto kReInitDistance = osm::object_id_type{1024};

  while (q_it != end(queries)) {
    auto const query_id = std::abs(q_it->first);

    switch (state) {
      case fsm_state::from_index: {
        auto const* it_idx = std::lower_bound(
            std::begin(idx), std::end(idx), query_id,
            [](auto const& o, auto const& i) { return o.id_ < i; });
        utl::verify(!(it_idx == std::begin(idx) && it_idx->id_ != query_id),
                    "missing (cannot happen)");
        if (it_idx == std::end(idx) || it_idx->id_ != query_id) {
          --it_idx;
        }
        if (block_exhausted &&
            it_idx->block_offset_ == last_block_offset) {
          ++q_it;
          break;
        }
        block_exhausted = false;
        last_block_offset = it_idx->block_offset_;
        curr_id = it_idx->id_;
        dat_it = dat_base + it_idx->block_offset_ + it_idx->in_block_offset_;
        state = fsm_state::at_span_start;
      } break;

      case fsm_state::at_span_start: {
        auto const header = pz::decode_varint(&dat_it, dat_global_end);
        span_size = static_cast<std::int64_t>(header >> 1U);
        if ((header & 0x1U) == 0x1U) {
          if (span_size == 0) {
            // end-of-block: re-enter `from_index` for the next query
            block_exhausted = true;
            state = fsm_state::from_index;
            break;
          }
          // empty-span (in-block gap)
          curr_id += static_cast<osm::object_id_type>(span_size);
          state = fsm_state::at_span_start;
          break;
        }
        span_size += 1;
        span_pos = 0;

        if (query_id < curr_id) {
          for (; q_it != end(queries) && std::abs(q_it->first) < curr_id;
               ++q_it) {
          }
          x_dec.reset(static_cast<fixed_coord_t>(read_fixed(&dat_it)));
          y_dec.reset(static_cast<fixed_coord_t>(read_fixed(&dat_it)));
          state = fsm_state::in_span;
          break;
        }
        if (query_id < curr_id + span_size) {
          x_dec.reset(static_cast<fixed_coord_t>(read_fixed(&dat_it)));
          y_dec.reset(static_cast<fixed_coord_t>(read_fixed(&dat_it)));
          state = fsm_state::in_span;
          break;
        }

        // skip past this span
        dat_it += 2 * sizeof(std::uint32_t);
        for (auto i = std::int64_t{1}; i < span_size; ++i) {
          pz::skip_varint(&dat_it, dat_global_end);
          pz::skip_varint(&dat_it, dat_global_end);
        }
        curr_id += static_cast<osm::object_id_type>(span_size);
        state = fsm_state::at_span_start;
      } break;

      case fsm_state::in_span: {
        if (query_id < curr_id + (span_size - span_pos)) {
          while (curr_id != query_id) {
            utl::verify(span_pos < span_size, "hit end of span");
            x_dec.decode(static_cast<fixed_delta_t>(pz::decode_zigzag64(
                pz::decode_varint(&dat_it, dat_global_end))));
            y_dec.decode(static_cast<fixed_delta_t>(pz::decode_zigzag64(
                pz::decode_varint(&dat_it, dat_global_end))));
            ++curr_id;
            ++span_pos;
          }
          utl::verify(query_id == curr_id, "missed node");
          for (; q_it != end(queries) && std::abs(q_it->first) == query_id;
               ++q_it) {
            q_it->second->set_x(static_cast<std::int32_t>(x_dec.curr_));
            q_it->second->set_y(static_cast<std::int32_t>(y_dec.curr_));
          }
          state = fsm_state::in_span;
          break;
        }

        if (query_id > curr_id + kReInitDistance) {
          state = fsm_state::from_index;
          break;
        }

        // Unwind the rest of the current span.
        ++span_pos;
        for (; span_pos < span_size; ++span_pos) {
          pz::skip_varint(&dat_it, dat_global_end);
          pz::skip_varint(&dat_it, dat_global_end);
          ++curr_id;
        }
        ++curr_id;
        state = fsm_state::at_span_start;
      } break;
    }
  }
}

void update_locations_of_way(hybrid_node_idx const& nodes, osm::way& way) {
  // Reused per thread: this is called once per way (millions of times) and the
  // vector is pure scratch (pointers into `way`'s own nodes), so there is no
  // reason to re-allocate it every time.
  static thread_local std::vector<std::pair<osm::object_id_type, osm::location*>>
      query;
  query.clear();
  query.reserve(way.nodes().size());
  for (auto& nr : way.nodes()) {
    query.emplace_back(nr.ref(), &nr.location());
  }
  if (query.empty()) {
    return;
  }
  get_coords(nodes, query);
  for (auto const& p : query) {
    p.second->set_x(p.second->x() - hybrid_node_idx::x_offset);
    p.second->set_y(p.second->y() - hybrid_node_idx::y_offset);
  }
}

void update_locations(hybrid_node_idx const& nodes, std::span<osm::way> ways) {
  static thread_local std::vector<std::pair<osm::object_id_type, osm::location*>>
      query;
  query.clear();
  for (auto& w : ways) {
    for (auto& nr : w.nodes()) {
      query.emplace_back(nr.ref(), &nr.location());
    }
  }
  get_coords(nodes, query);
  for (auto const& p : query) {
    p.second->set_x(p.second->x() - hybrid_node_idx::x_offset);
    p.second->set_y(p.second->y() - hybrid_node_idx::y_offset);
  }
}

hybrid_node_idx::hybrid_node_idx(cista::mmap idx_mmap, cista::mmap dat_mmap)
    : impl_{std::make_unique<impl>(std::move(idx_mmap), std::move(dat_mmap))} {}
hybrid_node_idx::~hybrid_node_idx() = default;

// ----- per-block encoder (parallel) ------------------------------------

namespace {

inline void write_fixed_u32(std::string& out, std::uint32_t v) {
  for (auto i = 0U; i < sizeof(v); ++i) {
    out.push_back(static_cast<char>(v & 0xffU));
    v >>= 8U;
  }
}

inline void write_varint(std::string& out, std::uint64_t v) {
  pz::write_varint(std::back_inserter(out), v);
}

void emit_sub_spans(hybrid_block& result,
                    std::vector<fixed_xy>& span,
                    osm::object_id_type const last_id) {
  if (span.empty()) {
    return;
  }
  auto const first_id_in_span =
      last_id - static_cast<osm::object_id_type>(span.size()) + 1;
  delta_encoder x_enc{0};
  delta_encoder y_enc{0};
  for (auto i = std::size_t{0U}; i < span.size();) {
    x_enc.reset(span[i].x());
    y_enc.reset(span[i].y());

    auto j = i + 1U;
    for (; j < span.size(); ++j) {
      auto const sx =
          get_varint_size(pz::encode_zigzag64(x_enc.encode(span[j].x())));
      auto const sy =
          get_varint_size(pz::encode_zigzag64(y_enc.encode(span[j].y())));
      if (sx + sy > 2 * sizeof(std::uint32_t) + 1U) {
        break;
      }
    }
    auto const span_size = j - i;
    result.span_starts_.push_back(hybrid_block::span_start{
        first_id_in_span + static_cast<osm::object_id_type>(i),
        result.encoded_.size(), result.total_coords_});

    write_varint(result.encoded_,
                 ((static_cast<std::uint64_t>(span_size) - 1U) << 1U) | 0x0U);
    write_fixed_u32(result.encoded_, static_cast<std::uint32_t>(span[i].x()));
    write_fixed_u32(result.encoded_, static_cast<std::uint32_t>(span[i].y()));

    x_enc.reset(span[i].x());
    y_enc.reset(span[i].y());

    ++i;
    for (; i < j; ++i) {
      write_varint(result.encoded_,
                   pz::encode_zigzag64(x_enc.encode(span[i].x())));
      write_varint(result.encoded_,
                   pz::encode_zigzag64(y_enc.encode(span[i].y())));
    }
    result.total_coords_ += span_size;
  }
  span.clear();
}

}  // namespace

void hybrid_block_encoder::push(osm::object_id_type const id,
                                fixed_xy const& pos) {
  constexpr auto coord_min = std::numeric_limits<std::uint32_t>::min();
  constexpr auto coord_max = std::numeric_limits<std::uint32_t>::max();
  utl::verify(pos.x() >= coord_min && pos.y() >= coord_min &&
                  pos.x() <= coord_max && pos.y() <= coord_max,
              "pos ({}, {}) not within bounds ({} / {})", pos.x(), pos.y(),
              coord_min, coord_max);

  auto const abs_id = std::abs(id);
  if (abs_id == last_id_) {
    utl::verify(
        pos == last_pos_,
        "input: duplicate absolute node id with mismatching coordinates {}",
        abs_id);
    return;
  }
  utl::verify(abs_id > last_id_, "input: node ids are not sorted! {} <= {}",
              abs_id, last_id_);

  if (result_.first_id_ == std::numeric_limits<osm::object_id_type>::min()) {
    result_.first_id_ = abs_id;
  }

  if (last_id_ + 1 != abs_id && !span_.empty()) {
    emit_sub_spans(result_, span_, last_id_);
    write_varint(
        result_.encoded_,
        (static_cast<std::uint64_t>(abs_id - last_id_ - 1) << 1U) | 0x1U);
  }

  last_id_ = abs_id;
  last_pos_ = pos;
  span_.emplace_back(pos);
}

hybrid_block hybrid_block_encoder::finish() && {
  emit_sub_spans(result_, span_, last_id_);
  result_.last_id_ = last_id_;
  // End-of-block marker: zero-size empty-span varint (single byte 0x01).
  // The walker stops here, so blocks can be concatenated in `dat_` without
  // any inter-block stitching.
  if (!result_.encoded_.empty()) {
    write_varint(result_.encoded_, std::uint64_t{1U});
  }
  return std::move(result_);
}

// ----- merger (single-fiber, in input order) --------------------------

struct hybrid_block_merger::impl {
  explicit impl(hybrid_node_idx& nodes)
      : idx_{nodes.impl_->idx_}, dat_{nodes.impl_->dat_} {}

  void merge(hybrid_block&& block) {
    if (block.encoded_.empty()) {
      return;
    }

    auto const block_offset = static_cast<std::uint64_t>(dat_.size());
    auto const payload = block.encoded_.size();

    dat_.resize_uninitialized(block_offset + payload);
    std::memcpy(reinterpret_cast<std::uint8_t*>(dat_.data()) + block_offset,
                block.encoded_.data(), payload);

    auto first_in_block = true;
    for (auto const& ss : block.span_starts_) {
      auto const cum_coords = global_coords_written_ + ss.coords_before_;
      if (first_in_block || idx_.empty() ||
          cum_coords - last_idx_coord_ >= kCoordsPerIndex) {
        idx_.push_back(id_offset{
            ss.start_id_, block_offset,
            static_cast<std::uint32_t>(ss.offset_in_encoded_)});
        last_idx_coord_ = cum_coords;
        first_in_block = false;
      }
    }

    global_coords_written_ += block.total_coords_;
  }

  void finish() {}

  cista::basic_mmap_vec<id_offset, std::uint64_t>& idx_;
  cista::basic_mmap_vec<char, std::uint64_t>& dat_;
  std::size_t global_coords_written_{0U};
  std::size_t last_idx_coord_{0U};
};

hybrid_block_merger::hybrid_block_merger(hybrid_node_idx& nodes)
    : impl_{std::make_unique<impl>(nodes)} {}
hybrid_block_merger::~hybrid_block_merger() = default;

void hybrid_block_merger::merge(hybrid_block&& block) {
  impl_->merge(std::move(block));
}
void hybrid_block_merger::finish() { impl_->finish(); }

}  // namespace osm
