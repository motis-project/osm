#pragma once

#include <iostream>

#include "osm/types.h"

namespace osm {

struct problem_reporter {
  explicit problem_reporter(std::ostream* out = nullptr, bool debug = false)
      : out_stream_{out}, report{debug} {}

  void header(const char* msg) {
    if (!out_stream_) {
      return;
    }
    *out_stream_ << "DATA PROBLEM: " << msg << " ON ";
  }

  void report_duplicate_node(object_id_type node_id1,
                             object_id_type node_id2,
                             location location) {
    if (!out_stream_) {
      return;
    }
    header("duplicate node");
    *out_stream_ << "node_id1=" << node_id1 << " node_id2=" << node_id2
                 << " location=" << location.x() << "," << location.y() << "\n";
  }

  void report_touching_ring(object_id_type node_id, location location) {
    if (!out_stream_) {
      return;
    }
    header("touching ring");
    *out_stream_ << "node_id=" << node_id << " location=" << location.x() << ","
                 << location.y() << "\n";
  }

  void report_intersection(object_id_type way1_id,
                           location way1_seg_start,
                           location way1_seg_end,
                           object_id_type way2_id,
                           location way2_seg_start,
                           location way2_seg_end,
                           location intersection) {
    if (!out_stream_) {
      return;
    }
    header("intersection");
    *out_stream_ << "way1_id=" << way1_id
                 << " way1_seg_start=" << way1_seg_start.x() << ","
                 << way1_seg_start.y() << " way1_seg_end=" << way1_seg_end.x()
                 << "," << way1_seg_end.y() << " way2_id=" << way2_id
                 << " way2_seg_start=" << way2_seg_start.x() << ","
                 << way2_seg_start.y() << " way2_seg_end=" << way2_seg_end.x()
                 << "," << way2_seg_end.y()
                 << " intersection=" << intersection.x() << ","
                 << intersection.y() << "\n";
  }

  void report_duplicate_segment(const node_ref& nr1, const node_ref& nr2) {
    if (!out_stream_) {
      return;
    }
    header("duplicate segment");
    *out_stream_ << "node_id1=" << nr1.ref()
                 << " location1=" << nr1.location().x() << ","
                 << nr1.location().y() << " node_id2=" << nr2.ref()
                 << " location2=" << nr2.location().x() << ","
                 << nr2.location().y() << "\n";
  }

  void report_overlapping_segment(const node_ref& nr1, const node_ref& nr2) {
    if (!out_stream_) {
      return;
    }
    header("overlapping segment");
    *out_stream_ << "node_id1=" << nr1.ref()
                 << " location1=" << nr1.location().x() << ","
                 << nr1.location().y() << " node_id2=" << nr2.ref()
                 << " location2=" << nr2.location().x() << ","
                 << nr2.location().y() << "\n";
  }

  void report_ring_not_closed(const node_ref& nr, const way* way) {
    if (!out_stream_) {
      return;
    }
    header("ring not closed");
    *out_stream_ << "node_id=" << nr.ref() << " location=" << nr.location().x()
                 << "," << nr.location().y();
    if (way) {
      *out_stream_ << " on way " << way->id;
    }
    *out_stream_ << "\n";
  }

  void report_role_should_be_outer(object_id_type way_id,
                                   location seg_start,
                                   location seg_end) {
    if (!out_stream_) {
      return;
    }
    header("role should be outer");
    *out_stream_ << "way_id=" << way_id << " seg_start=" << seg_start.x() << ","
                 << seg_start.y() << " seg_end=" << seg_end.x() << ","
                 << seg_end.y() << "\n";
  }

  void report_role_should_be_inner(object_id_type way_id,
                                   location seg_start,
                                   location seg_end) {
    if (!out_stream_) {
      return;
    }
    header("role should be inner");
    *out_stream_ << "way_id=" << way_id << " seg_start=" << seg_start.x() << ","
                 << seg_start.y() << " seg_end=" << seg_end.x() << ","
                 << seg_end.y() << "\n";
  }

  void report_way_in_multiple_rings(const way& way) {
    if (!out_stream_) {
      return;
    }
    header("way in multiple rings");
    *out_stream_ << "way_id=" << way.id << '\n';
  }

  void report_inner_with_same_tags(const way& way) {
    if (!out_stream_) {
      return;
    }
    header("inner way with same tags as relation or outer");
    *out_stream_ << "way_id=" << way.id << '\n';
  }

  void report_invalid_location(object_id_type way_id, object_id_type node_id) {
    if (!out_stream_) {
      return;
    }
    header("invalid location");
    *out_stream_ << "way_id=" << way_id << " node_id=" << node_id << '\n';
  }

  void report_duplicate_way(const way& way) {
    if (!out_stream_) {
      return;
    }
    header("duplicate way");
    *out_stream_ << "way_id=" << way.id << '\n';
  }

  bool report = false;
  std::ostream* out_stream_;
};
}  // namespace osm