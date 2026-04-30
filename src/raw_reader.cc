#include "osm/raw_reader.h"

#include <cstring>

#include "protozero/pbf_message.hpp"
#include "protozero/types.hpp"

#include "cista/endian/conversion.h"

#include "utl/verify.h"

#include "osm/tags.h"

namespace osm {

constexpr auto const kMaxBlobHeaderSize = 64U * 1024U;
constexpr auto const kMaxUncompressedBlobSize = 32U * 1024U * 1024U;

std::optional<buf> raw_reader::read() {
  if (rest_.empty()) {
    return std::nullopt;
  }

  auto const read = [&](std::size_t n) {
    utl::verify(n <= rest_.size(), "bytes left {} < {}", rest_.size(), n);
    auto buf = rest_.substr(0, n);
    rest_ = rest_.substr(n);
    return buf;
  };

  // Read blob header size.
  auto size = std::uint32_t{};
  auto size_buf = read(sizeof(size));
  std::memcpy(&size, size_buf.data(), sizeof(size));
  if constexpr (cista::endian_conversion_necessary<
                    cista::mode::SERIALIZE_BIG_ENDIAN>()) {
    size = cista::endian_swap(size);
  }
  utl::verify(size <= kMaxBlobHeaderSize, "blob header size {} >= {}", size,
              kMaxBlobHeaderSize);

  // Parse blob header.
  auto data_size = 0;
  auto blob_header_type = protozero::data_view{};
  auto hdr = protozero::pbf_message<tag::blob_header>{read(size)};
  while (hdr.next()) {
    switch (hdr.tag_and_type()) {
      case protozero::tag_and_type(tag::blob_header::kRequiredStringType,
                                   protozero::pbf_wire_type::length_delimited):
        blob_header_type = hdr.get_view();
        break;

      case protozero::tag_and_type(tag::blob_header::kRequiredInt32Datasize,
                                   protozero::pbf_wire_type::varint):
        data_size = hdr.get_int32();
        break;

      default: hdr.skip();
    }
  }

  // Parse blob.
  auto raw_size = 0;
  auto compressed = std::optional<std::string_view>{};
  auto blob = protozero::pbf_message<tag::blob_header>{read(data_size)};
  while (blob.next()) {
    switch (blob.tag_and_type()) {
      case protozero::tag_and_type(tag::blob::kOptionalBytesZlibData,
                                   protozero::pbf_wire_type::length_delimited):
        compressed = blob.get_view();
        break;

      case protozero::tag_and_type(tag::blob::kOptionalInt32RawSize,
                                   protozero::pbf_wire_type::varint):
        raw_size = blob.get_int32();
        utl::verify(raw_size >= 0 && raw_size <= kMaxUncompressedBlobSize,
                    "invalid raw size {}", raw_size);
        break;

      default: blob.skip();
    }
  }
  utl::verify(compressed.has_value(), "unsupported blob type");

  return buf{static_cast<unsigned>(raw_size), *compressed};
}

void raw_reader::reset() noexcept { rest_ = file_.view(); }

}  // namespace osm
