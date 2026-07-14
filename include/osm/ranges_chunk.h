#pragma once

// Minimal drop-in replacement for std::views::chunk (P2442R1).
//
// Some standard libraries (notably libc++ as of clang 21) do not yet ship
// std::ranges::chunk_view / std::views::chunk. This header provides a small,
// self-contained substitute that is *only* activated when the standard
// library does not advertise the feature via __cpp_lib_ranges_chunk. When the
// real facility is available, this header is empty and std::views::chunk is
// used unchanged.
//
// The implementation covers just what osm::decode_dense_nodes (osm/decoder.h)
// needs:
//
//     range | std::views::chunk(n) | std::views::transform(...)
//
// i.e. a const-iterable view that groups the elements of a forward source
// range into consecutive chunks of at most `n` elements. It intentionally does
// not model the sized / random-access / bidirectional refinements of the
// standard view.

#include <version>

#if !defined(__cpp_lib_ranges_chunk)

#include <iterator>
#include <ranges>
#include <type_traits>
#include <utility>

namespace osm::ranges_chunk_compat {

template <std::ranges::view V>
  requires std::ranges::input_range<V> && std::ranges::input_range<V const>
class chunk_view : public std::ranges::view_interface<chunk_view<V>> {
  using diff_t = std::ranges::range_difference_t<V const>;
  using It = std::ranges::iterator_t<V const>;
  using Sent = std::ranges::sentinel_t<V const>;

  V base_ = V();
  diff_t n_ = 1;

  // A single chunk: at most `remaining_` elements, iterated from a copy of the
  // enclosing chunk's start iterator (so consuming a chunk never disturbs the
  // outer position). A plain range (begin/end) is all the transform step needs.
  struct chunk_range {
    It cur_ = It();
    Sent end_ = Sent();
    diff_t remaining_ = 0;

    struct iterator {
      using iterator_concept = std::input_iterator_tag;
      using iterator_category = std::input_iterator_tag;
      using value_type = std::iter_value_t<It>;
      using difference_type = diff_t;

      It cur_ = It();
      Sent end_ = Sent();
      diff_t remaining_ = 0;

      constexpr decltype(auto) operator*() const { return *cur_; }
      constexpr iterator& operator++() {
        ++cur_;
        --remaining_;
        return *this;
      }
      constexpr void operator++(int) { ++*this; }

      constexpr bool operator==(std::default_sentinel_t) const {
        return remaining_ <= 0 || cur_ == end_;
      }
    };

    constexpr iterator begin() const { return {cur_, end_, remaining_}; }
    constexpr std::default_sentinel_t end() const { return {}; }
  };

public:
  struct iterator {
    using iterator_concept = std::input_iterator_tag;
    using iterator_category = std::input_iterator_tag;
    using value_type = chunk_range;
    using difference_type = diff_t;

    It cur_ = It();
    Sent end_ = Sent();
    diff_t n_ = 1;

    constexpr value_type operator*() const { return {cur_, end_, n_}; }

    constexpr iterator& operator++() {
      std::ranges::advance(cur_, n_, end_);
      return *this;
    }
    constexpr void operator++(int) { ++*this; }

    constexpr bool operator==(std::default_sentinel_t) const {
      return cur_ == end_;
    }
  };

  chunk_view()
    requires std::default_initializable<V>
  = default;
  constexpr chunk_view(V base, diff_t const n)
      : base_{std::move(base)}, n_{n} {}

  constexpr iterator begin() const {
    return {std::ranges::begin(base_), std::ranges::end(base_), n_};
  }
  constexpr std::default_sentinel_t end() const { return {}; }
};

template <class R>
chunk_view(R&&, std::ranges::range_difference_t<R>)
    -> chunk_view<std::views::all_t<R>>;

// --- range adaptor ---------------------------------------------------------

struct chunk_closure {
  std::ptrdiff_t n_;

  template <std::ranges::viewable_range R>
  friend constexpr auto operator|(R&& r, chunk_closure const c) {
    return chunk_view<std::views::all_t<R>>{
        std::views::all(std::forward<R>(r)),
        static_cast<std::ranges::range_difference_t<R>>(c.n_)};
  }
};

struct chunk_fn {
  constexpr chunk_closure operator()(std::ptrdiff_t const n) const {
    return chunk_closure{n};
  }

  template <std::ranges::viewable_range R>
  constexpr auto operator()(R&& r,
                            std::ranges::range_difference_t<R> const n) const {
    return chunk_view<std::views::all_t<R>>{std::views::all(std::forward<R>(r)),
                                            n};
  }
};

}  // namespace osm::ranges_chunk_compat

// The one intrusive part: decoder.h calls std::views::chunk unchanged, so the
// name has to resolve there. std::views is an alias for std::ranges::views;
// inject the adaptor object only on standard libraries that lack it.
namespace std::ranges::views {
inline constexpr ::osm::ranges_chunk_compat::chunk_fn chunk{};
}  // namespace std::ranges::views

#endif  // !defined(__cpp_lib_ranges_chunk)
