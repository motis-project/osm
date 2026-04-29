#pragma once

#include <cassert>
#include <cstring>
#include <array>
#include <list>
#include <numeric>
#include <set>
#include <span>
#include <unordered_set>
#include <vector>

#include "ProblemReporter.h"
#include "assembler_stats.h"
#include "osm/types.h"
#include "vec.h"

namespace assembler {

struct ProtoRing;

enum role_type : uint8_t { unknown = 0, outer = 1, inner = 2, empty = 3 };
static role_type parse_role(const char* role) noexcept {
  if (role[0] == '\0') {
    return role_type::empty;
  }
  if (!std::strcmp(role, "outer")) {
    return role_type::outer;
  }
  if (!std::strcmp(role, "inner")) {
    return role_type::inner;
  }
  return role_type::unknown;
}

using open_ring_its_type = std::list<std::list<ProtoRing>::iterator>;
/**
 * This helper for the Assembler models a segment,
 * the connection between two nodes.
 *
 * Internally segments have their smaller coordinate at the
 * beginning of the segment. Smaller, in this case, means smaller
 * x coordinate, and, if they are the same, smaller y coordinate.
 */
struct NodeRefSegment {
  osm::NodeRef first_noderef_;
  osm::NodeRef second_noderef_;
  const osm::Way* way_ = nullptr;
  // The ring this segment is part of. Initially nullptr, this
  // will be filled in once we know which ring the segment is in.
  ProtoRing* ring_ = nullptr;
  role_type role_ = role_type::unknown;
  // Nodes have to be reversed to get the intended order.
  bool reverse_ = false;
  // We found the right direction for this segment in the ring.
  // (This depends on whether it is an inner or outer ring.)
  bool direction_done_ = false;

  NodeRefSegment() noexcept = default;
  NodeRefSegment(const osm::NodeRef& nr1,
                 const osm::NodeRef& nr2,
                 role_type role,
                 const osm::Way* way) noexcept
      : first_noderef_(nr1.location() < nr2.location() ? nr1 : nr2),
        second_noderef_(nr1.location() < nr2.location() ? nr2 : nr1),
        way_(way),
        role_(role) {}

  ProtoRing* ring() const noexcept { return ring_; }
  bool is_done() const noexcept { return ring_ != nullptr; }
  void set_ring(ProtoRing* ring) noexcept {
    assert(ring);
    ring_ = ring;
  }

  bool is_reverse() const noexcept { return reverse_; }
  void reverse() noexcept { reverse_ = !reverse_; }
  bool is_direction_done() const noexcept { return direction_done_; }
  void mark_direction_done() noexcept { direction_done_ = true; }
  void mark_direction_not_done() noexcept { direction_done_ = false; }

  const osm::NodeRef& first() const noexcept { return first_noderef_; }
  const osm::NodeRef& second() const noexcept { return second_noderef_; }
  const osm::NodeRef& start() const noexcept {
    return reverse_ ? second_noderef_ : first_noderef_;
  }
  const osm::NodeRef& stop() const noexcept {
    return reverse_ ? first_noderef_ : second_noderef_;
  }

  bool role_outer() const noexcept { return role_ == role_type::outer; }
  bool role_inner() const noexcept { return role_ == role_type::inner; }
  bool role_empty() const noexcept { return role_ == role_type::empty; }
  const char* role_name() const noexcept {
    static const std::array<const char*, 4> names = {
        {"unknown", "outer", "inner", "empty"}};
    return names[static_cast<int>(role_)];
  }
  const osm::Way* way() const noexcept { return way_; }

  /**
   * The "determinant" of this segment. Used for calculating
   * the winding order of a ring.
   */
  int64_t det() const noexcept {
    const vec a{start()};
    const vec b{stop()};
    return a * b;
  }
  bool operator==(const NodeRefSegment& other) const noexcept {
    return first_noderef_.location() == other.first().location() &&
           second_noderef_.location() == other.second().location();
  }
  bool operator!=(const NodeRefSegment& other) const noexcept {
    return !(*this == other);
  }
  /**
   * A NodeRefSegment is "smaller" if the first point is to the
   * left and down of the first point of the second segment.
   * If both first points are the same, the segment with the higher
   * slope comes first. If the slope is the same, the shorter
   * segment comes first.
   */
  bool operator<(const NodeRefSegment& other) const noexcept {
    if (first_noderef_.location() == other.first().location()) {
      const vec p0{first_noderef_.location()};
      const vec p1{second_noderef_.location()};
      const vec q0{other.first().location()};
      const vec q1{other.second().location()};
      const vec p = p1 - p0;
      const vec q = q1 - q0;
      if (p.x == 0 && q.x == 0) {
        return p.y < q.y;
      }
      const auto a = p.y * q.x;
      const auto b = q.y * p.x;
      if (a == b) {
        return p.x < q.x;
      }
      return a > b;
    }
    return first_noderef_.location() < other.first().location();
  }
};  // struct NodeRefSegment

inline bool outside_x_range(const NodeRefSegment& s1,
                            const NodeRefSegment& s2) noexcept {
  return s1.first().location().x() > s2.second().location().x();
}

inline bool y_range_overlap(const NodeRefSegment& s1,
                            const NodeRefSegment& s2) noexcept {
  const std::pair<std::int32_t, std::int32_t> m1 =
      std::minmax(s1.first().location().y(), s1.second().location().y());
  const std::pair<std::int32_t, std::int32_t> m2 =
      std::minmax(s2.first().location().y(), s2.second().location().y());
  return !(m1.first > m2.second || m2.first > m1.second);
}
/**
 * Calculate the intersection between two NodeRefSegments. The
 * result is returned as a Location. Note that because the Location
 * uses integers with limited precision internally, the result
 * might be slightly different than the numerically correct
 * location.
 *
 * This function uses integer arithmetic as much as possible and
 * will not work if the segments are longer than about half the
 * planet. This shouldn't happen with real data, so it isn't a big
 * problem.
 *
 * If the segments touch in one or both of their endpoints, it
 * doesn't count as an intersection.
 *
 * If the segments intersect not in a single point but in multiple
 * points, ie if they are collinear and overlap, the smallest
 * of the endpoints that is in the overlapping section is returned.
 *
 * @returns Undefined osmium::Location if there is no intersection
 *          or a defined Location if the segments intersect.
 */
inline osm::Location calculate_intersection(const NodeRefSegment& s1,
                                            const NodeRefSegment& s2) noexcept {
  // See
  // https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  // for some hints about how the algorithm works.
  const vec p0{s1.first()};
  const vec p1{s1.second()};
  const vec q0{s2.first()};
  const vec q1{s2.second()};

  if ((p0 == q0 && p1 == q1) || (p0 == q1 && p1 == q0)) {
    // segments are the same
    return osm::Location{};
  }

  const vec pd = p1 - p0;
  const std::int64_t d = pd * (q1 - q0);
  if (d != 0) {
    // segments are not collinear
    if (p0 == q0 || p0 == q1 || p1 == q0 || p1 == q1) {
      // touching at an end point
      return osm::Location{};
    }
    // intersection in a point
    const std::int64_t na =
        ((q1.x - q0.x) * (p0.y - q0.y)) - ((q1.y - q0.y) * (p0.x - q0.x));
    const std::int64_t nb =
        ((p1.x - p0.x) * (p0.y - q0.y)) - ((p1.y - p0.y) * (p0.x - q0.x));
    if ((d > 0 && na >= 0 && na <= d && nb >= 0 && nb <= d) ||
        (d < 0 && na <= 0 && na >= d && nb <= 0 && nb >= d)) {
      const double ua = static_cast<double>(na) / static_cast<double>(d);
      const vec i = p0 + ua * (p1 - p0);
      return osm::Location{static_cast<int32_t>(i.x),
                           static_cast<int32_t>(i.y)};
    }
    return osm::Location{};
  }

  // segments are collinear
  if (pd * (q0 - p0) == 0) {
    // segments are on the same line
    struct seg_loc {
      int segment;
      osm::Location location;
    };
    std::array<seg_loc, 4UL> sl = {{
        {0, s1.first().location()},
        {0, s1.second().location()},
        {1, s2.first().location()},
        {1, s2.second().location()},
    }};
    std::sort(sl.begin(), sl.end(), [](const seg_loc& lhs, const seg_loc& rhs) {
      return lhs.location < rhs.location;
    });
    if (sl[1].location == sl[2].location) {
      return osm::Location{};
    }
    if (sl[0].segment != sl[1].segment) {
      if (sl[0].location == sl[1].location) {
        return sl[2].location;
      }
      return sl[1].location;
    }
  }
  return osm::Location{};
}

struct ProtoRing {
  std::vector<NodeRefSegment*> segments_;
  std::vector<ProtoRing*> inner_;
  NodeRefSegment* min_segment_;
  ProtoRing* outer_ring_ = nullptr;
  std::int64_t sum_ = 0;

  explicit ProtoRing(NodeRefSegment* segment) noexcept : min_segment_(segment) {
    add_segment_back(segment);
  }

  void add_segment_back(NodeRefSegment* segment) {
    assert(segment);
    if (*segment < *min_segment_) {
      min_segment_ = segment;
    }
    segments_.push_back(segment);
    segment->set_ring(this);
    sum_ += segment->det();
  }

  NodeRefSegment* min_segment() const noexcept { return min_segment_; }
  ProtoRing* outer_ring() const noexcept { return outer_ring_; }
  const std::vector<ProtoRing*>& inner_rings() const noexcept { return inner_; }

  void set_outer_ring(ProtoRing* outer_ring) noexcept {
    assert(outer_ring);
    assert(inner_.empty());
    outer_ring_ = outer_ring;
  }

  void add_inner_ring(ProtoRing* ring) {
    assert(ring);
    assert(!outer_ring_);
    inner_.push_back(ring);
  }

  bool is_outer() const noexcept { return !outer_ring_; }

  const std::vector<NodeRefSegment*>& segments() const noexcept {
    return segments_;
  }
  const osm::NodeRef& get_node_ref_start() const noexcept {
    return segments_.front()->start();
  }
  const osm::NodeRef& get_node_ref_stop() const noexcept {
    return segments_.back()->stop();
  }

  bool closed() const noexcept {
    return get_node_ref_start().location() == get_node_ref_stop().location();
  }

  void reverse() {
    std::for_each(segments_.begin(), segments_.end(),
                  [](NodeRefSegment* segment) { segment->reverse(); });
    std::reverse(segments_.begin(), segments_.end());
    sum_ = -sum_;
  }

  void mark_direction_done() {
    std::for_each(
        segments_.begin(), segments_.end(),
        [](NodeRefSegment* segment) { segment->mark_direction_done(); });
  }

  bool is_cw() const noexcept { return sum_ <= 0; }
  int64_t sum() const noexcept { return sum_; }

  void fix_direction() noexcept {
    if (is_cw() == is_outer()) {
      reverse();
    }
  }

  void reset() {
    inner_.clear();
    outer_ring_ = nullptr;
    std::for_each(
        segments_.begin(), segments_.end(),
        [](NodeRefSegment* segment) { segment->mark_direction_not_done(); });
  }

  void get_ways(std::set<const osm::Way*>& ways) const {
    for (const auto& segment : segments_) {
      ways.insert(segment->way());
    }
  }

  void join_forward(ProtoRing& other) {
    segments_.reserve(segments_.size() + other.segments_.size());
    for (NodeRefSegment* segment : other.segments_) {
      add_segment_back(segment);
    }
  }

  void join_backward(ProtoRing& other) {
    segments_.reserve(segments_.size() + other.segments_.size());
    for (auto it = other.segments_.rbegin(); it != other.segments_.rend();
         ++it) {
      (*it)->reverse();
      add_segment_back(*it);
    }
  }

  void print(std::ostream& out) const {
    out << "Ring [";
    if (!segments_.empty()) {
      out << segments_.front()->start().ref();
    }
    for (const auto& segment : segments_) {
      out << ',' << segment->stop().ref();
    }
    out << "]-" << (is_outer() ? "OUTER" : "INNER");
  }

};  // struct ProtoRing

struct SegmentList {
  std::vector<NodeRefSegment> segments_;
  std::vector<osm::object_id_type> relations_missing_ways;
  /**
   * Calculate the number of segments in all the ways together.
   */
  static std::size_t get_num_segments(
      const std::vector<const osm::Way*>& members) noexcept {
    return std::accumulate(members.cbegin(), members.cend(),
                           static_cast<std::size_t>(0),
                           [](std::size_t sum, const osm::Way* way) {
                             if (way->nodes().empty()) {
                               return sum;
                             }
                             return sum + way->nodes().size() - 1;
                           });
  }

  uint32_t extract_segments_from_way_impl(ProblemReporter& problem_reporter,
                                          uint64_t& duplicate_nodes,
                                          const osm::Way& way,
                                          role_type role) {
    uint32_t invalid_locations = 0;
    osm::NodeRef previous_nr;
    for (const osm::NodeRef& nr : way.nodes()) {
      if (!nr.location().valid()) {
        ++invalid_locations;
        if (problem_reporter.report) {
          problem_reporter.report_invalid_location(way.id, nr.ref());
        }
        continue;
      }
      if (previous_nr.location().is_set()) {
        if (previous_nr.location() != nr.location()) {
          segments_.emplace_back(previous_nr, nr, role, &way);
        } else {
          ++duplicate_nodes;
          if (problem_reporter.report) {
            problem_reporter.report_duplicate_node(previous_nr.ref(), nr.ref(),
                                                   nr.location());
          }
        }
      }
      previous_nr = nr;
    }
    return invalid_locations;
  }

  SegmentList() = default;
  SegmentList(const SegmentList&) = delete;
  SegmentList(SegmentList&&) = delete;

  SegmentList& operator=(const SegmentList&) = delete;
  SegmentList& operator=(SegmentList&&) = delete;

  ~SegmentList() noexcept = default;

  std::size_t size() const noexcept { return segments_.size(); }
  bool empty() const noexcept { return segments_.empty(); }
  NodeRefSegment& front() { return segments_.front(); }
  NodeRefSegment& back() { return segments_.back(); }

  const NodeRefSegment& operator[](std::size_t n) const noexcept {
    assert(n < segments_.size());
    return segments_[n];
  }

  NodeRefSegment& operator[](const std::size_t n) noexcept {
    assert(n < segments_.size());
    return segments_[n];
  }

  auto begin() noexcept { return segments_.begin(); }
  auto end() noexcept { return segments_.end(); }
  auto begin() const noexcept { return segments_.begin(); }
  auto end() const noexcept { return segments_.end(); }
  void sort() { std::sort(segments_.begin(), segments_.end()); }

  /**
   * Extract segments from given way and add them to the list.
   * Segments connecting two nodes with the same location (ie
   * same node or different nodes with same location) are
   * removed after reporting the duplicate node.
   */
  uint32_t extract_segments_from_way(ProblemReporter& problem_reporter,
                                     uint64_t& duplicate_nodes,
                                     const osm::Way& way) {
    if (way.nodes().empty()) {
      return 0;
    }
    segments_.reserve(way.nodes().size() - 1);
    return extract_segments_from_way_impl(problem_reporter, duplicate_nodes,
                                          way, role_type::outer);
  }

  /**
   * Extract all segments from all ways that make up this
   * multipolygon relation and add them to the list.
   * Changed for loop, because relations are not filtered
   */
  template <typename Members>
  uint32_t extract_segments_from_ways(
      ProblemReporter& problem_reporter,
      uint64_t& duplicate_nodes,
      uint64_t& duplicate_ways,
      const osm::Relation<Members>& relation,
      const std::vector<const osm::Way*>& ways) {
    assert(relation.members().size() >= ways.size());

    const std::size_t num_segments = get_num_segments(ways);
    segments_.reserve(num_segments);

    std::unordered_set<osm::object_id_type> ids;
    ids.reserve(ways.size());
    uint32_t invalid_locations = 0;
    auto way_not_found = 0;
    for (const auto& [member_id, member_role, member_type] :
         relation.members()) {
      if (member_type == osm::member_type::kWay) {
        auto found_it = std::find_if(
            ways.begin(), ways.end(),
            [member_id](const osm::Way* w) { return w->id == member_id; });
        if (found_it != ways.end()) {
          if (ids.count((*found_it)->id) == 0) {
            ids.insert((*found_it)->id);
            const auto role = parse_role(member_role.data());
            invalid_locations += extract_segments_from_way_impl(
                problem_reporter, duplicate_nodes, **found_it, role);
          } else {
            ++duplicate_ways;
            if (problem_reporter.report) {
              problem_reporter.report_duplicate_way(**found_it);
            }
          }
        } else {
          way_not_found++;
        }
      }
    }
    if (way_not_found > 0) {
      relations_missing_ways.push_back(relation.id);
    }
    return invalid_locations;
  }

  /**
   * Find duplicate segments (ie same start and end point) in the
   * list and remove them. This will always remove pairs of the
   * same segment. So if there are three, for instance, two will
   * be removed and one will be left.
   */
  void erase_duplicate_segments(ProblemReporter* problem_reporter,
                                uint64_t& duplicate_segments,
                                uint64_t& overlapping_segments) {
    while (true) {
      auto size = segments_.size();
      auto it = std::adjacent_find(segments_.begin(), segments_.end());
      if (it == segments_.end()) {
        break;
      }
      // Only count and report duplicate segments if they
      // belong to the same way or if they don't both have
      // the role "inner". Those cases are definitely wrong.
      // If the duplicate segments belong to different
      // "inner" ways, they could be touching inner rings
      // which are perfectly okay. Note that for this check
      // the role has to be correct in the member data.
      if (it->way() == std::next(it)->way() || !it->role_inner() ||
          !std::next(it)->role_inner()) {
        ++duplicate_segments;
        if (problem_reporter) {
          problem_reporter->report_duplicate_segment(it->first(), it->second());
        }
      }
      // if (it + 2 != segments_.end() && *it == *(it + 2)) {
      if (it + 2 != segments_.end() && it == (it + 2)) {
        ++overlapping_segments;
        if (problem_reporter) {
          problem_reporter->report_overlapping_segment(it->first(),
                                                       it->second());
        }
      }
      segments_.erase(it, it + 2);
    }
  }

  /**
   * Find intersection between segments.
   *
   * @param problem_reporter Any intersections found are
   *                         reported to this object.
   * @returns true if there are intersections.
   */
  uint32_t find_intersections(ProblemReporter* problem_reporter) const {
    if (segments_.empty()) {
      return 0;
    }
    uint32_t found_intersections = 0;
    for (auto it1 = segments_.cbegin(); it1 != segments_.cend() - 1; ++it1) {
      const NodeRefSegment& s1 = *it1;
      for (auto it2 = it1 + 1; it2 != segments_.end(); ++it2) {
        const NodeRefSegment& s2 = *it2;
        // erase_duplicate_segments() should have made sure of that
        assert(s1 != s2);

        if (outside_x_range(s2, s1)) {
          break;
        }
        if (y_range_overlap(s1, s2)) {
          const osm::Location intersection{calculate_intersection(s1, s2)};
          if (intersection.is_set()) {
            ++found_intersections;
            if (problem_reporter) {
              problem_reporter->report_intersection(
                  s1.way()->id, s1.first().location(), s1.second().location(),
                  s2.way()->id, s2.first().location(), s2.second().location(),
                  intersection);
            }
          }
        }
      }
    }
    return found_intersections;
  }
};  // struct SegmentList

struct location_to_ring_map {
  osm::Location location;
  open_ring_its_type::iterator ring_it;
  bool start{false};

  location_to_ring_map(osm::Location l,
                       open_ring_its_type::iterator r,
                       const bool s) noexcept
      : location(l), ring_it(r), start(s) {}

  explicit location_to_ring_map(osm::Location l) noexcept : location(l) {}
  const ProtoRing& ring() const noexcept { return **ring_it; }

  bool operator==(const location_to_ring_map& other) const {
    return location == other.location;
  }
  bool operator<(const location_to_ring_map& other) const {
    return location < other.location;
  }
};  // struct location_to_ring_map

struct candidate {
  int64_t sum;
  std::vector<std::pair<location_to_ring_map, bool>> rings;
  osm::Location start_location;
  osm::Location stop_location;
  explicit candidate(location_to_ring_map& ring, bool reverse)
      : sum(ring.ring().sum()),
        start_location(ring.ring().get_node_ref_start().location()),
        stop_location(ring.ring().get_node_ref_stop().location()) {
    rings.emplace_back(ring, reverse);
  }
  bool closed() const noexcept { return start_location == stop_location; }
};

struct slocation {
  enum { invalid_item = 1U << 30U };
  std::uint32_t item : 31;
  std::uint32_t reverse : 1;
  slocation() noexcept : item(invalid_item), reverse(false) {}
  explicit slocation(uint32_t n, bool r = false) noexcept
      : item(n), reverse(r) {}
  osm::Location location(const SegmentList& segment_list) const noexcept {
    const auto& segment = segment_list[item];
    return reverse ? segment.second().location() : segment.first().location();
  }
  const osm::NodeRef& node_ref(const SegmentList& segment_list) const noexcept {
    const auto& segment = segment_list[item];
    return reverse ? segment.second() : segment.first();
  }
  osm::Location location(const SegmentList& segment_list,
                         const osm::Location& default_location) const noexcept {
    if (item == invalid_item) {
      return default_location;
    }
    return location(segment_list);
  }
};  // struct slocation

struct rings_stack_element {
  double m_y;
  ProtoRing* m_ring_ptr;

  rings_stack_element(double y, ProtoRing* ring_ptr)
      : m_y(y), m_ring_ptr(ring_ptr) {}
  double y() const noexcept { return m_y; }
  const ProtoRing& ring() const noexcept { return *m_ring_ptr; }
  ProtoRing* ring_ptr() noexcept { return m_ring_ptr; }
  bool operator==(const rings_stack_element& rhs) const noexcept {
    return m_ring_ptr == rhs.m_ring_ptr;
  }
  bool operator<(const rings_stack_element& rhs) const noexcept {
    return m_y < rhs.m_y;
  }
};  // struct rings_stack_element

/**
 * The area_pair struct stores all rings in one vector.
 * "offsets" marks where rings begin.
 * offsets[0] is always the outer ring.
 */
struct area_pair {
  std::vector<osm::NodeRef> area_part;
  std::vector<std::int64_t> offsets;

  std::span<const osm::NodeRef> get_outer() const {
    if (offsets.empty()) return {};
    std::size_t start = offsets[0];
    std::size_t end = (offsets.size() > 1) ? offsets[1] : area_part.size();
    return {area_part.data() + start, end - start};
  }

  std::span<const osm::NodeRef> get_inner_at(std::size_t i) const {
    if (offsets.size() <= i + 1) return {};
    std::size_t start = offsets[i + 1];
    std::size_t end =
        (i + 2 < offsets.size()) ? offsets[i + 2] : area_part.size();
    return {area_part.data() + start, end - start};
  }

  std::span<const osm::NodeRef> get_inners() const {
    if (offsets.size() <= 1) return {};
    std::size_t start = offsets[1];
    std::size_t end = area_part.size();
    return {area_part.data() + start, end - start};
  }

  void rotate_to_smallest() {
    // outer:
    auto start_outer = offsets[0];
    auto end_outer = (offsets.size() > 1) ? offsets[1] : area_part.size();
    auto end = area_part.begin() + end_outer - 1;
    auto min_it =
        std::min_element(area_part.begin(), end,
                         [](const osm::NodeRef& a, const osm::NodeRef& b) {
                           return a.ref() < b.ref();
                         });
    auto rotation_offset = std::distance(area_part.begin(), min_it);
    std::rotate(area_part.begin(), area_part.begin() + rotation_offset, end);
    area_part[end - area_part.begin()] = area_part[0];
    //  inners:
    for (auto i = 1; i < offsets.size(); ++i) {
      auto start_inner = offsets[i];
      auto it_start = area_part.begin() + start_inner;
      auto end_inner =
          (i + 1 < offsets.size()) ? offsets[i + 1] : area_part.size();
      auto it_end = area_part.begin() + end_inner - 1;
      auto it_min = std::min_element(
          it_start, it_end, [](const osm::NodeRef& a, const osm::NodeRef& b) {
            return a.ref() < b.ref();
          });
      auto rotation_offset = std::distance(it_start, it_min);
      std::rotate(it_start, it_start + rotation_offset, it_end);
      area_part[it_end - area_part.begin()] = *it_start;
    }
  }

  void reverse_outer_orientation() {
    auto start_outer = offsets[0];
    auto end_outer = (offsets.size() > 1) ? offsets[1] : area_part.size();
    assert(start_outer == 0);
    std::reverse(area_part.begin(), area_part.begin() + end_outer);
  }

  void reverse_inners_orientation() {
    if (offsets.size() <= 1) {
      return;
    }
    for (auto i = 1; i < offsets.size(); ++i) {
      auto start_inner = offsets[i];
      auto end_inner =
          (i + 1 < offsets.size()) ? offsets[i + 1] : area_part.size();
      std::reverse(area_part.begin() + start_inner,
                   area_part.begin() + end_inner);
    }
  }

  void reverse_complete_orientation() {
    reverse_outer_orientation();
    if (offsets.size() <= 1) {
      return;
    }
    reverse_inners_orientation();
  }
};

/**
 * The (multi)-polygon area.
 * If from_way is true => only one area_pair
 */
struct polygon_area {
  std::int64_t origin_id;
  bool valid = false;
  std::vector<area_pair> area;
  bool missing_flag = false;  // indicates missing ways in dataset
  bool from_way = false;  // indicates that this polygon was built from a way

  assembler::area_stats pa_stats{};

  explicit polygon_area(std::int64_t id) : origin_id(id) {}

  std::vector<std::span<const osm::NodeRef>> get_all_outers() {
    std::vector<std::span<const osm::NodeRef>> result;
    for (auto const& ap : area) {
      result.push_back(ap.get_outer());
    }
    return result;
  }

  std::vector<std::span<const osm::NodeRef>> get_inners_seperated(
      std::size_t outer_index) {
    std::vector<std::span<const osm::NodeRef>> result;
    if (outer_index >= area.size()) return result;
    auto const& ap = area[outer_index];
    result.reserve(ap.offsets.size() - 1);
    for (std::size_t i = 0; i + 1 < ap.offsets.size(); ++i) {
      result.push_back(ap.get_inner_at(i));
    }
    return result;
  }
};

}  // namespace assembler
