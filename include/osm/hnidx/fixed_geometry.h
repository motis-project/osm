#pragma once

#include <cstdint>
#include <tuple>

namespace osm {

using fixed_coord_t = std::int64_t;
using fixed_delta_t = std::int64_t;

struct fixed_xy {
  constexpr fixed_coord_t x() const noexcept { return x_; }
  constexpr fixed_coord_t y() const noexcept { return y_; }
  constexpr bool operator==(fixed_xy const&) const noexcept = default;

  fixed_coord_t x_;
  fixed_coord_t y_;
};

}  // namespace osm
