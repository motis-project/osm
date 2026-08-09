#pragma once

#include <cstddef>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

#include "boost/fiber/all.hpp"
#include "boost/fiber/protected_fixedsize_stack.hpp"

#include "utl/parser/buf_reader.h"

#include "osm/decoder.h"
#include "osm/inflate.h"
#include "osm/raw_reader.h"
#include "osm/work_stealing.h"

namespace osm {

struct noop_flush {
  void operator()(auto&) const noexcept {}
};

// Parse a .osm.pbf stream in parallel.
//
// `make_local` is invoked once per worker fiber to produce a fiber-local
// state object; this object is forwarded as the first argument to each
// handler. Use `[]{ return std::monostate{}; }` (or any factory) when no
// per-fiber state is needed. Handlers are invoked concurrently across
// fibers, but each fiber's `Local` is only accessed by that fiber, so it
// needs no synchronization.
//
// Handler signatures:
//   on_node (Local&, std::int64_t id, geo::latlng const& pos, auto&& tags)
//   on_way  (Local&, std::int64_t id, auto&& refs,            auto&& tags)
//   on_rel  (Local&, std::int64_t id, auto&& members,         auto&& tags)
//   on_flush(Local&)
//
// `on_flush` is invoked once per decoded PBF block on the fiber that decoded
// it, right after all of that block's primitives have been handled. It lets a
// handler buffer objects during the block and process them in a single batch
// at the block boundary (e.g. to resolve all of a block's node locations with
// one sorted index walk). Any `string_view`s handed to the primitive handlers
// point into the block and stay valid until (and including) this call.
//
// `progress_consumer` is invoked with the cumulative number of input bytes
// consumed so far; defaults to a no-op.
template <typename LocalFactory,
          typename NodeFn,
          typename WayFn,
          typename RelFn,
          typename FlushFn = noop_flush,
          typename ProgressConsumer = utl::noop_progress_consumer>
void parse_osm(raw_reader& r,
               LocalFactory&& make_local,
               NodeFn&& on_node,
               WayFn&& on_way,
               RelFn&& on_rel,
               FlushFn&& on_flush = FlushFn{},
               ProgressConsumer&& progress_consumer = ProgressConsumer{},
               unsigned const n_threads = std::thread::hardware_concurrency(),
               unsigned const n_fibers = 0U) {
  namespace bf = boost::fibers;

  // A handler passed as `skip` means "don't decode this primitive type".
  constexpr auto read_nodes = !std::is_same_v<std::decay_t<NodeFn>, skip>;
  constexpr auto read_ways = !std::is_same_v<std::decay_t<WayFn>, skip>;
  constexpr auto read_relations = !std::is_same_v<std::decay_t<RelFn>, skip>;

  // Default to one fiber per thread. Pass a larger `n_fibers` (e.g. 4× the
  // thread count) to oversubscribe — useful when handlers prefetch+sleep
  // and would otherwise leave OS threads spinning in `pick_next` while the
  // currently-running fiber is blocked on a page fault.
  auto const fiber_count = n_fibers == 0U ? n_threads : n_fibers;

  auto group = work_stealing_group{n_threads + 1U};
  auto pool = std::vector<std::thread>{n_threads};
  auto fin = std::atomic_bool{false};
  auto fin_cv = bf::condition_variable_any{};
  auto fin_mutex = std::mutex{};

  static constexpr auto kFiberStackSize = std::size_t{8U * 1024U * 1024U};
  auto stack_allocator = bf::protected_fixedsize_stack{kFiberStackSize};

  auto ch = bf::buffered_channel<buf>{64U};
  auto fibers = std::vector<bf::fiber>{};
  fibers.reserve(fiber_count);
  for (auto i = 0U; i != fiber_count; ++i) {
    fibers.emplace_back(std::allocator_arg, stack_allocator, [&]() {
      auto local = make_local();
      auto decompressor = inflate{};
      auto out = std::string{};
      auto strings = std::vector<std::string_view>{};

      for (auto const& b : ch) {
        out.resize(b.raw_size_);
        decompressor.decompress(b.compressed_, out);
        decode_primitive<read_nodes, read_ways, read_relations>(
            out, strings,
            [&](auto&& a, auto&& b, auto&& c) {
              on_node(local, std::forward<decltype(a)>(a),
                      std::forward<decltype(b)>(b),
                      std::forward<decltype(c)>(c));
            },
            [&](auto&& a, auto&& b, auto&& c) {
              on_way(local, std::forward<decltype(a)>(a),
                     std::forward<decltype(b)>(b),
                     std::forward<decltype(c)>(c));
            },
            [&](auto&& a, auto&& b, auto&& c) {
              on_rel(local, std::forward<decltype(a)>(a),
                     std::forward<decltype(b)>(b),
                     std::forward<decltype(c)>(c));
            });
        on_flush(local);
      }
    });
  }

  for (auto& t : pool) {
    t = std::thread{[&]() {
      bf::use_scheduling_algorithm<work_stealing>(group, n_threads + 1U);
      auto l = std::unique_lock{fin_mutex};
      fin_cv.wait(l, [&]() { return fin.load(); });
    }};
  }

  // Main thread joins as the (n+1)-th participant — must happen AFTER the
  // host pool has been spawned (the barrier inside `work_stealing` waits for
  // n+1 participants).
  bf::use_scheduling_algorithm<work_stealing>(group, n_threads + 1U);

  auto next = std::optional<buf>{};
  while ((next = r.read()).has_value()) {
    ch.push(*next);
    progress_consumer(r.file_.size() - r.rest_.size());
  }
  ch.close();

  // Join fibers before tearing down host-thread schedulers; otherwise a fiber
  // still in `suspend_until` may dereference a freed algo_.
  for (auto& f : fibers) {
    f.join();
  }

  fin.store(true);
  fin_cv.notify_all();

  for (auto& t : pool) {
    t.join();
  }
}

}  // namespace osm
