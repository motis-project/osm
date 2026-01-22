#pragma once

#include "boost/fiber/all.hpp"

namespace osm {

template <typename NodeFn, typename WayFn, typename RelFn>
void decode_primitive_parallel(osm::raw_reader& r,
                               bool const read_nodes,
                               bool const read_ways,
                               bool const read_relations,
                               NodeFn&& on_node,
                               WayFn&& on_way,
                               RelFn&& on_rel,
                               utl::progress_tracker_ptr pt) {
  namespace bf = boost::fibers;

  auto const n_threads = std::thread::hardware_concurrency();

  auto ch = bf::buffered_channel<osm::buf>{64U};
  for (auto i = 0U; i != n_threads; ++i) {
    bf::fiber([&]() {
      auto decompressor = osm::inflate{};
      auto out = std::string{};
      auto strings = std::vector<std::string_view>{};

      for (auto const& b : ch) {
        out.resize(b.raw_size_);
        decompressor.decompress(b.compressed_, out);

        osm::decode_primitive(out, strings, true, true, true, on_node, on_way,
                              on_rel);
      }
    }).detach();
  }

  auto pool = std::vector<std::thread>{n_threads};
  auto fin = std::atomic_bool{false};
  auto fin_cv = bf::condition_variable_any{};
  auto fin_mutex = std::mutex{};

  for (auto& t : pool) {
    t = std::thread{[&]() {
      bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads + 1U);
      auto l = std::unique_lock{fin_mutex};
      fin_cv.wait(l, [&]() { return fin.load(); });
    }};
  }

  bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads + 1U);

  auto buf = std::optional<osm::buf>{};
  while ((buf = r.read()).has_value()) {
    ch.push(*buf);
    pt->update(r.file_.size() - r.rest_.size());
  }
  ch.close();
  fin.store(true);
  fin_cv.notify_all();

  for (auto& t : pool) {
    t.join();
  }
}

}  // namespace osm