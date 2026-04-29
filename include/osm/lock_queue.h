#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

namespace osm {

template <typename T>
struct LockQueue {
  std::queue<std::shared_ptr<T>> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::atomic<bool> done_ = false;

  LockQueue() = default;

  void push(T value) {
    auto ptr = std::make_shared<T>(std::move(value));
    {
      std::lock_guard<std::mutex> lock(mtx_);
      queue_.push(ptr);
    }
    cv_.notify_one();
  }

  std::shared_ptr<T> pop() {
    std::unique_lock<std::mutex> lock(mtx_);
    // wake up if either not empty OR done
    cv_.wait(lock, [&] { return !queue_.empty() || done_.load(); });
    // done.load() true & empty => nullptr
    if (queue_.empty()) return nullptr;
    auto item = queue_.front();
    queue_.pop();
    return item;
  }

  void set_done() {
    done_.store(true);
    cv_.notify_all();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return queue_.empty();
  }
};

}  // namespace osm