#pragma once

#include <cinttypes>
#include <iterator>

#include "utl/helpers/algorithm.h"

#include "protozero/varint.hpp"

namespace osm {

template <typename T = std::int64_t, bool Delta = false>
struct varint {
  struct iterator {
    using iterator_concept = std::forward_iterator_tag;
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = std::int64_t;
    using pointer = std::int64_t const*;
    using reference = std::int64_t const&;

    iterator() = default;
    explicit iterator(std::string_view const d) : pos_{d} {
      if (pos_.empty()) {
        pos_ = {};  // past-the-end iterator
      } else {
        decode();
      }
    }

    reference operator*() const { return value_; }
    pointer operator->() const { return &value_; }

    iterator& operator++() {
      pos_ = next_;
      if (pos_.empty()) {
        pos_ = {};  // canonical past-the-end position
      } else {
        decode();
      }
      return *this;
    }

    iterator operator++(int) {
      auto const tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(iterator const& a, iterator const& b) {
      return a.pos_.data() == b.pos_.data() && a.pos_.size() == b.pos_.size();
    }

  private:
    void decode() {
      auto start = pos_.data();
      auto const end = start + pos_.size();
      auto const prev = value_;
      auto const x = protozero::decode_varint(&start, end);
      if constexpr (std::is_signed_v<T>) {
        if constexpr (sizeof(value_type) == 4) {
          value_ = protozero::decode_zigzag32(x);
        } else {
          static_assert(sizeof(value_type) == 8);
          value_ = protozero::decode_zigzag64(x);
        }
      } else {
        value_ = static_cast<T>(x);
      }
      if constexpr (Delta) {
        value_ += prev;
      }
      next_ = std::string_view{start, static_cast<std::size_t>(end - start)};
    }

    std::string_view pos_{};  // bytes from the current element on; {} == end
    std::string_view next_{};  // bytes after the current element (cached)
    value_type value_{0};
  };

  iterator begin() const { return iterator{data_}; }
  iterator end() const { return iterator{}; }

  bool empty() const noexcept { return data_.empty(); }

  std::size_t size() const noexcept {
    return utl::count_if(data_, [](char c) noexcept {
      return (static_cast<unsigned char>(c) & 0x80U) == 0U;
    });
  }

  std::string_view data_;
};

template <typename T>
using delta_varint = varint<T, true>;

}  // namespace osm