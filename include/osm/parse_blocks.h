#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "utl/parallel_for.h"

#include "osm/decoder.h"
#include "osm/inflate.h"
#include "osm/raw_reader.h"

namespace osm {

// Decompress + decode already-read PBF `blocks` in parallel, producing one
// `Context` per block and handing them to `collect` in strict block (file)
// order.
//
// For each block, on some worker thread:
//   1. the block is inflated into a thread-local scratch buffer,
//   2. `decode_primitive` walks it, invoking `on_node` / `on_way` / `on_rel`
//      for the requested primitive types with a freshly default-constructed
//      `Context&` as their first argument,
//   3. `on_flush(Context&)` runs once, after all of the block's primitives.
// The resulting `Context` is then moved to `collect(i, Context&&)`, which is
// invoked single-threaded and in ascending block order. `progress(i)` is
// invoked (unordered) as each block finishes decoding.
//
// Pass `osm::skip{}` for any primitive handler whose type should not be
// decoded at all; the block is then walked without touching those primitives.
//
// `Context` is the caller's per-block state: the handlers accumulate the
// decoded data into it and it carries that data to `collect`. It must be
// default-constructible and movable. The inflate/decode scratch
// (decompressor, decompressed bytes, string table) is owned by this function
// and reused per worker thread; the `Context` only holds what the caller
// needs downstream.
//
// Handler signatures (each receives the block's `Context&` first):
//   on_node (Context&, std::int64_t id, geo::latlng const& pos, auto&& tags)
//   on_way  (Context&, std::int64_t id, auto&& refs,            auto&& tags)
//   on_rel  (Context&, std::int64_t id, auto&& members,         auto&& tags)
//   on_flush(Context&)
//   collect (std::size_t i, Context&&)
//   progress(std::size_t i)
//
// `string_view`s handed to the primitive handlers point into the decoded
// block and stay valid until (and including) `on_flush`, but not in
// `collect` -- copy anything needed later into the `Context`.
template <typename Context,
          typename NodeFn,
          typename WayFn,
          typename RelFn,
          typename FlushFn,
          typename Collect,
          typename ProgressFn>
void parse_osm_block_parallel_collect(std::vector<buf> const& blocks,
                                      NodeFn&& on_node,
                                      WayFn&& on_way,
                                      RelFn&& on_rel,
                                      FlushFn&& on_flush,
                                      Collect&& collect,
                                      ProgressFn&& progress) {
  // A handler passed as `skip` means "don't decode this primitive type".
  constexpr auto read_nodes = !std::is_same_v<std::decay_t<NodeFn>, skip>;
  constexpr auto read_ways = !std::is_same_v<std::decay_t<WayFn>, skip>;
  constexpr auto read_relations = !std::is_same_v<std::decay_t<RelFn>, skip>;

  struct block_local {
    inflate decompressor_;
    std::string decompressed_;
    std::vector<std::string_view> strings_;
  };

  utl::parallel_ordered_collect_threadlocal<block_local>(
      blocks.size(),
      [&](block_local& local, std::size_t const i) -> Context {
        auto ctx = Context{};
        local.decompressed_.resize(blocks[i].raw_size_);
        local.decompressor_.decompress(blocks[i].compressed_,
                                       local.decompressed_);
        // decode_primitive always calls the handlers with exactly three
        // arguments (id, position/refs/members, tags). Spelling that fixed
        // arity out -- rather than forwarding a variadic `auto&&...` pack --
        // matters for compile time: MSVC instantiates variadic generic lambdas
        // with pack-forwarding pathologically slowly, and these bridge lambdas
        // are instantiated once per parse pass.
        decode_primitive<read_nodes, read_ways, read_relations>(
            local.decompressed_, local.strings_,
            [&](auto&& a, auto&& b, auto&& c) {
              on_node(ctx, std::forward<decltype(a)>(a),
                      std::forward<decltype(b)>(b),
                      std::forward<decltype(c)>(c));
            },
            [&](auto&& a, auto&& b, auto&& c) {
              on_way(ctx, std::forward<decltype(a)>(a),
                     std::forward<decltype(b)>(b),
                     std::forward<decltype(c)>(c));
            },
            [&](auto&& a, auto&& b, auto&& c) {
              on_rel(ctx, std::forward<decltype(a)>(a),
                     std::forward<decltype(b)>(b),
                     std::forward<decltype(c)>(c));
            });
        on_flush(ctx);
        return ctx;
      },
      std::forward<Collect>(collect), std::forward<ProgressFn>(progress));
}

}  // namespace osm
