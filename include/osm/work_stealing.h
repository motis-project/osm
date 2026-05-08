#pragma once

#include <atomic>
#include <barrier>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <vector>

#include "boost/intrusive_ptr.hpp"

#include "boost/fiber/algo/algorithm.hpp"
#include "boost/fiber/context.hpp"
#include "boost/fiber/detail/context_spinlock_queue.hpp"
#include "boost/fiber/type.hpp"

#include "boost/context/detail/prefetch.hpp"

namespace osm {

class work_stealing;

// Per-`parse_osm`-call state shared across all `work_stealing` schedulers in
// the pool. Replaces the upstream `static` state in
// `boost::fibers::algo::work_stealing` so multiple calls to
// `use_scheduling_algorithm<work_stealing>` in the same process don't alias
// or use-after-free. Group lifetime must extend past all participating
// threads.
struct work_stealing_group {
  explicit work_stealing_group(std::uint32_t const n)
      : schedulers_(n, nullptr), barrier_(n) {}

  std::atomic<std::uint32_t> counter_{0U};
  std::vector<boost::intrusive_ptr<work_stealing>> schedulers_;
  std::barrier<> barrier_;
};

// Direct copy of `boost::fibers::algo::work_stealing` with the static
// counter / scheduler vector / thread barrier replaced by per-call
// `work_stealing_group` state. Per-thread ready queues with random victim
// stealing — same scheduling semantics as upstream.
class work_stealing : public boost::fibers::algo::algorithm {
public:
  work_stealing(work_stealing_group& group,
                std::uint32_t const thread_count,
                bool const suspend = false)
      : group_{group},
        id_{group_.counter_.fetch_add(1U)},
        thread_count_{thread_count},
        suspend_{suspend} {
    group_.schedulers_[id_] = this;
    group_.barrier_.arrive_and_wait();
  }

  work_stealing(work_stealing const&) = delete;
  work_stealing(work_stealing&&) = delete;
  work_stealing& operator=(work_stealing const&) = delete;
  work_stealing& operator=(work_stealing&&) = delete;

  void awakened(boost::fibers::context* ctx) noexcept override {
    if (!ctx->is_context(boost::fibers::type::pinned_context)) {
      ctx->detach();
    }
    rqueue_.push(ctx);
  }

  boost::fibers::context* pick_next() noexcept override {
    auto* victim = rqueue_.pop();
    if (victim != nullptr) {
      boost::context::detail::prefetch_range(
          victim, sizeof(boost::fibers::context));
      if (!victim->is_context(boost::fibers::type::pinned_context)) {
        boost::fibers::context::active()->attach(victim);
      }
    } else {
      auto const size = group_.schedulers_.size();
      auto count = std::size_t{0};
      static thread_local auto generator =
          std::minstd_rand{std::random_device{}()};
      auto distribution = std::uniform_int_distribution<std::uint32_t>{
          0U, static_cast<std::uint32_t>(thread_count_ - 1U)};
      auto id = std::uint32_t{0};
      do {
        do {
          ++count;
          // random selection of one logical cpu;
          // prevent stealing from own scheduler.
          id = distribution(generator);
        } while (id == id_);
        victim = group_.schedulers_[id]->steal();
      } while (victim == nullptr && count < size);
      if (victim != nullptr) {
        boost::context::detail::prefetch_range(
            victim, sizeof(boost::fibers::context));
        boost::fibers::context::active()->attach(victim);
      }
    }
    return victim;
  }

  boost::fibers::context* steal() noexcept { return rqueue_.steal(); }

  bool has_ready_fibers() const noexcept override { return !rqueue_.empty(); }

  void suspend_until(std::chrono::steady_clock::time_point const& time_point)
      noexcept override {
    if (!suspend_) {
      return;
    }
    auto lk = std::unique_lock{mtx_};
    if (time_point == std::chrono::steady_clock::time_point::max()) {
      cnd_.wait(lk, [this] { return flag_; });
    } else {
      cnd_.wait_until(lk, time_point, [this] { return flag_; });
    }
    flag_ = false;
  }

  void notify() noexcept override {
    if (!suspend_) {
      return;
    }
    auto lk = std::unique_lock{mtx_};
    flag_ = true;
    lk.unlock();
    cnd_.notify_all();
  }

private:
  work_stealing_group& group_;
  std::uint32_t id_;
  std::uint32_t thread_count_;
  // The spinlock-protected queue's brief contention on `steal()` doubles as a
  // natural backoff for empty-stealing — replacing it with the lock-free
  // SPMC queue removed the backoff and turned `pick_next` into a busy spin
  // when all queues were drained.
  boost::fibers::detail::context_spinlock_queue rqueue_{};
  std::mutex mtx_{};
  std::condition_variable cnd_{};
  bool flag_{false};
  bool suspend_;
};

}  // namespace osm
