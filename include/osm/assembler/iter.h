#pragma once

#include <type_traits>
#include <utility>

namespace assembler {

template <typename It, typename P = std::pair<It, It>>
struct iterator_range : public P {

  using iterator = It;

  explicit iterator_range(P&& p) noexcept : P(std::move(p)) {}

  It begin() const noexcept { return this->first; }
  It end() const noexcept { return this->second; }

  bool empty() const noexcept { return begin() == end(); }

};  // struct iterator_range

/**
 * Helper function to create iterator_range from std::pair.
 */
template <typename P, typename It = typename P::first_type>
inline iterator_range<It> make_range(P&& p) noexcept {
  static_assert(std::is_same<P, std::pair<It, It>>::value,
                "make_range needs pair of iterators as argument");
  return iterator_range<It>{std::forward<P>(p)};
}
}  // namespace assembler