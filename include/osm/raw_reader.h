#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "cista/mmap.h"

namespace osm {

struct buf {
  std::size_t raw_size_;
  std::string_view compressed_;
};

struct raw_reader {
  std::optional<buf> read();

  void reset() noexcept;

  cista::mmap file_;
  std::string_view rest_{file_.view()};
};

}  // namespace osm
