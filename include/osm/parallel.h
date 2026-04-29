#pragma once

#include <string_view>

#include "osm.h"
#include "osm/decoder.h"
#include "osm/hnidx/hybrid_node_index.h"
#include "osm/inflate.h"
#include "osm/lock_queue.h"
#include "osm/memory.h"

#include "utl/progress_tracker.h"

namespace osm {

template <typename NodeFn, typename WayFn, typename RelFn>
void decode_primitive_parallel(
    osm::raw_reader& r,
    tiles::hybrid_node_idx_builder& node_index_builder,
    bool const read_nodes,
    bool const read_ways,
    bool const read_relations,
    NodeFn&& on_node,
    WayFn&& on_way,
    RelFn&& on_rel,
    utl::progress_tracker_ptr pt) {

  auto const n_threads = std::thread::hardware_concurrency();
  auto pool = std::vector<std::thread>{n_threads};

  std::atomic<std::uint64_t> highest_node_bid{0};
  std::atomic<std::uint64_t> next_block_id{0};
  struct WorkItem {
    std::uint64_t block_id;
    osm::buf buffer;
  };
  struct MergeItem {
    std::uint64_t block_id;
    std::shared_ptr<std::vector<osm::Node>> nodes;
  };

  auto merge_queue = osm::LockQueue<MergeItem>{};
  auto prod_queue = osm::LockQueue<WorkItem>{};
  for (auto& t : pool) {
    t = std::thread{[&]() {
      auto decompressor = osm::inflate{};
      auto out = std::string{};
      auto strings = std::vector<std::string_view>{};

      auto local_nodes = std::make_shared<std::vector<osm::Node>>();
      local_nodes->reserve(10000);

      while (auto work = prod_queue.pop()) {
        local_nodes->clear();
        auto on_node_local = [&](std::int64_t const id, geo::latlng const& pos,
                                 auto&& tags) {
          osm::Location temp_loc = osm::Location(pos.lat(), pos.lng());
          osm::Node temp_node{id, temp_loc};
          local_nodes->emplace_back(temp_node);
        };

        out.resize(work->buffer.raw_size_);
        decompressor.decompress(work->buffer.compressed_, out);
        osm::decode_primitive(out, strings, read_nodes, read_ways,
                              read_relations, on_node_local, on_way, on_rel);
        if (read_nodes) {
          merge_queue.push({work->block_id, std::move(local_nodes)});
          local_nodes = std::make_shared<std::vector<osm::Node>>();
          local_nodes->reserve(10000);
        }
      }
    }};
  }

  std::thread merger_thread([&]() {
    if (read_nodes) {
      auto next_expected = std::uint64_t{0};
      int node_count = 0;
      int empty_node_count = 0;
      auto items_pending =
          std::unordered_map<std::uint64_t,
                             std::shared_ptr<std::vector<osm::Node>>>{};
      while (auto item = merge_queue.pop()) {
        items_pending.emplace(item->block_id, std::move(item->nodes));
        while (items_pending.count(next_expected)) {
          auto& vec = items_pending[next_expected];
          if (vec->empty()) {
            empty_node_count++;
          }
          for (const auto& node : *vec) {
            node_index_builder.node(node);
            node_count++;
          }
          items_pending.erase(next_expected);
          ++next_expected;
        }
      }
      node_index_builder.finish();
      std::cout << "number of nodes: " << node_count << std::endl;
      r.set_offset(next_block_id - empty_node_count);
    }
  });

  auto buf = std::optional<osm::buf>{};
  int num = 0;
  while ((buf = r.read()).has_value()) {
    int count = next_block_id++;
    if (count < r.get_offset() && !read_nodes) {
      continue;
    }
    num++;
    WorkItem item;
    item.block_id = count;
    item.buffer = std::move(*buf);
    prod_queue.push(std::move(item));
    pt->update(r.file_.size() - r.rest_.size());
  }
  prod_queue.set_done();

  for (auto& t : pool) {
    t.join();
  }
  merge_queue.set_done();
  merger_thread.join();

  // const ium::MemoryUsage memory;
  // std::cout << "\nMemory used: " << memory.peak() << " MBytes\n";
}

}  // namespace osm