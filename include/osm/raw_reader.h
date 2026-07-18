#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "cista/mmap.h"

namespace osm {

struct buf {
  std::size_t raw_size_;
  std::string_view compressed_;
};

struct raw_reader {
  // Opens `path` read-only and hints the kernel that the file will be read
  // strictly forward (sequential readahead, drop pages behind the cursor).
  explicit raw_reader(std::string const& path);

  std::optional<buf> read();

  void reset() noexcept;

  cista::mmap file_;
  std::string_view rest_{file_.view()};
};

}  // namespace osm
