#include "edge.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>

namespace graph {

Edge::Edge(VertexSptrConst start, VertexSptrConst end,
           std::vector<bezier::CurveUptr> &&curves)
    : start_(std::move(start)),
      end_(std::move(end)),
      curves_(std::move(curves)) {}

bool Edge::IsIntersect(const Edge &other) const {
  // Curves may have common start or end point. Such points are not considered
  // as intersection points.

  const auto start = start_->AsPoint();
  const auto end = end_->AsPoint();

  const auto starts_match = (*start_ == *other.start_);
  const auto start_mathes_end = (*start_ == *other.end_);
  const auto end_matches_start = (*end_ == *other.start_);
  const auto ends_match = (*end_ == *other.end_);

  std::cerr << *this << ' ' << other << ' ' << starts_match << start_mathes_end
            << end_matches_start << ends_match << std::endl;

  const auto curves_back = curves_.size() - 1;
  const auto other_curves_back = other.curves_.size() - 1;

  for (std::size_t i = 0; i <= curves_back; i++) {
    for (std::size_t j = 0; j <= other_curves_back; j++) {
      if (i == 0 && (j == 0 && starts_match ||
                     j == other_curves_back && start_mathes_end) ||
          i == curves_back && (j == 0 && end_matches_start ||
                               j == other_curves_back && ends_match)) {
        const auto intersection_points =
            curves_[i]->Intersect(*other.curves_[j]);
        std::cout << "intersection: ";
        for (const auto &p : intersection_points) {
          std::cout << p << ' ';
        }
        std::cout << std::endl;

        // const auto &point_to_compare = (i == 0 ? start : end);
        const auto is_approximately_equal = [&start,
                                             &end](const bezier::Point &other) {
          return start.IsInNeighborhood(other) || end.IsInNeighborhood(other);
        };

        if (std::find_if_not(
                intersection_points.begin(), intersection_points.end(),
                is_approximately_equal) != intersection_points.end()) {
          return true;
        };
      } else if (curves_[i]->IsIntersect(*other.curves_[j])) {
        std::cerr << i << " intersects " << j << std::endl;
        return true;
      }
    }
  }

  return false;
}

bool Edge::operator==(const Edge &other) const {
  if (*start_ != *other.start_ || *end_ != *other.end_ ||
      curves_.size() != other.curves_.size()) {
    return false;
  }

  for (auto i = 0; i < curves_.size(); i++) {
    if (*curves_[i] != *other.curves_[i]) {
      return false;
    }
  }

  return true;
}

std::ostream &operator<<(std::ostream &os, const Edge &edge) {
  for (std::size_t i = 0; i < edge.curves_.size(); i++) {
    std::cout << *edge.curves_[i];

    if (i + 1 < edge.curves_.size()) {
      std::cout << ", ";
    }
  }

  return os;
}

}  // namespace graph
