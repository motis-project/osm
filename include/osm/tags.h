#pragma once

#include <protozero/types.hpp>

namespace osm::tag {

enum class blob : protozero::pbf_tag_type {
  kOptionalBytesRaw = 1,
  kOptionalInt32RawSize = 2,
  kOptionalBytesZlibData = 3,
  kOptionalBytesLzmaData = 4,
  kOptionalBytesLz4Data = 6,
  kOptionalBytesZstdData = 7
};

enum class blob_header : protozero::pbf_tag_type {
  kRequiredStringType = 1,
  kOptionalBytesIndexdata = 2,
  kRequiredInt32Datasize = 3
};

enum class header_block : protozero::pbf_tag_type {
  kOptionalHeaderBBoxBbox = 1,
  kRepeatedStringRequiredFeatures = 4,
  kRepeatedStringOptionalFeatures = 5,
  kOptionalStringWritingprogram = 16,
  kOptionalStringSource = 17,
  kOptionalInt64OsmosisReplicationTimestamp = 32,
  kOptionalInt64OsmosisReplicationSequenceNumber = 33,
  kOptionalStringOsmosisReplicationBaseUrl = 34
};

enum class header_bbox : protozero::pbf_tag_type {
  kRequiredSint64Left = 1,
  kRequiredSint64Right = 2,
  kRequiredSint64Top = 3,
  kRequiredSint64Bottom = 4
};

enum class primitive_block : protozero::pbf_tag_type {
  kRequiredStringTableStringtable = 1,
  kRepeatedPrimitiveGroupPrimitivegroup = 2,
  kOptionalInt32Granularity = 17,
  kOptionalInt32DateGranularity = 18,
  kOptionalInt64LatOffset = 19,
  kOptionalInt64LonOffset = 20
};

enum class primitive_group : protozero::pbf_tag_type {
  kUnknown = 0,
  kRepeatedNodeNodes = 1,
  kOptionalDenseNodesDense = 2,
  kRepeatedWayWays = 3,
  kRepeatedRelationRelations = 4,
  kRepeatedChangeSetChangesets = 5
};

enum class string_table : protozero::pbf_tag_type { kRepeatedBytesS = 1 };

enum class info : protozero::pbf_tag_type {
  kOptionalInt32Version = 1,
  kOptionalInt64Timestamp = 2,
  kOptionalInt64Changeset = 3,
  kOptionalInt32Uid = 4,
  kOptionalUint32UserSid = 5,
  kOptionalBoolVisible = 6
};

enum class dense_info : protozero::pbf_tag_type {
  kPackedInt32Version = 1,
  kPackedSint64Timestamp = 2,
  kPackedSint64Changeset = 3,
  kPackedSint32Uid = 4,
  kPackedSint32UserSid = 5,
  kPackedBoolVisible = 6
};

enum class node : protozero::pbf_tag_type {
  kRequiredSint64Id = 1,
  kPackedUint32Keys = 2,
  kPackedUint32Vals = 3,
  kOptionalInfoInfo = 4,
  kRequiredSint64Lat = 8,
  kRequiredSint64Lon = 9
};

enum class dense_nodes : protozero::pbf_tag_type {
  kPackedSint64Id = 1,
  kOptionalDenseInfoDenseinfo = 5,
  kPackedSint64Lat = 8,
  kPackedSint64Lon = 9,
  kPackedInt32KeysVals = 10
};

enum class way : protozero::pbf_tag_type {
  kRequiredInt64Id = 1,
  kPackedUint32Keys = 2,
  kPackedUint32Vals = 3,
  kOptionalInfoInfo = 4,
  kPackedSint64Refs = 8,
  kPackedSint64Lat = 9,
  kPackedSint64Lon = 10
};

enum class relation : protozero::pbf_tag_type {
  kRequiredInt64Id = 1,
  kPackedUint32Keys = 2,
  kPackedUint32Vals = 3,
  kOptionalInfoInfo = 4,
  kPackedInt32RolesSid = 8,
  kPackedSint64Memids = 9,
  kPackedMemberTypeTypes = 10
};

}  // namespace osm::tag
