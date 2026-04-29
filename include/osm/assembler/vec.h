#pragma once

#include <cstdint>
#include <iosfwd>

#include "osm/types.h"

namespace assembler {
/**
 * This helper class models a 2D vector in the mathematical sense.
 * It uses 64 bit integers internally which has enough precision
 * for most operations with inputs based on 32 bit locations.
 */
struct vec {
  std::int64_t x;
  std::int64_t y;
  constexpr vec(std::int64_t a, std::int64_t b) noexcept : x(a), y(b) {}
  constexpr explicit vec(const osm::Location& l) noexcept
      : x(l.x()), y(l.y()) {}
  constexpr explicit vec(const osm::NodeRef& nr) noexcept
      : x(nr.location().x()), y(nr.location().y()) {}
};

constexpr vec operator+(const vec& lhs, const vec& rhs) noexcept {
  return vec{lhs.x + rhs.x, lhs.y + rhs.y};
}

constexpr vec operator-(const vec& lhs, const vec& rhs) noexcept {
  return vec{lhs.x - rhs.x, lhs.y - rhs.y};
}

constexpr std::int64_t operator*(const vec& lhs, const vec& rhs) noexcept {
  return (lhs.x * rhs.y) - (lhs.y * rhs.x);
}

constexpr vec operator*(double s, const vec& v) noexcept {
  return vec{static_cast<std::int64_t>(s * static_cast<double>(v.x)),
             static_cast<std::int64_t>(s * static_cast<double>(v.y))};
}

constexpr vec operator*(const vec& v, double s) noexcept {
  return vec{static_cast<std::int64_t>(s * static_cast<double>(v.x)),
             static_cast<std::int64_t>(s * static_cast<double>(v.y))};
}

constexpr bool operator==(const vec& lhs, const vec& rhs) noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

constexpr bool operator!=(const vec& lhs, const vec& rhs) noexcept {
  return !(lhs == rhs);
}

template <typename TChar, typename TTraits>
inline std::basic_ostream<TChar, TTraits>& operator<<(
    std::basic_ostream<TChar, TTraits>& out, const vec& v) {
  return out << '(' << v.x << ',' << v.y << ')';
}
}  // namespace assembler