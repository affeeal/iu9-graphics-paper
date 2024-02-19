#include "edge.hpp"

#include <algorithm>
#include <iostream>

#include "utils.hpp"

namespace graph {

Edge::Edge(VertexSptrConst start, VertexSptrConst end,
           std::vector<bezier::CurveUptr> &&curves)
    : start_(std::move(start)),
      end_(std::move(end)),
      curves_(std::move(curves)) {}

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

std::ostream &operator<<(std::ostream &os, const Edge &e) {
  std::cout << '{' << *e.start_ << ", " << *e.end_ << ", [";
  for (std::size_t i = 0; i < e.curves_.size() - 1; i++) {
    std::cout << *e.curves_[i] << ", ";
  }
  std::cout << *e.curves_.back() << "]}";

  return os;
}

bool Edge::IsIntersect(const Edge &e) const {
  const auto start = start_->AsPoint();
  const auto end = end_->AsPoint();

  const auto starts_match = (*start_ == *e.start_);
  const auto start_mathes_end = (*start_ == *e.end_);
  const auto end_matches_start = (*end_ == *e.start_);
  const auto ends_match = (*end_ == *e.end_);

  const auto back_curve = curves_.size() - 1;
  const auto e_back_curve = e.curves_.size() - 1;

  for (std::size_t i = 0; i < curves_.size(); i++) {
    for (std::size_t j = 0; j < e.curves_.size(); j++) {
      if (i == 0 && (j == 0 && starts_match ||
                     j == e_back_curve && start_mathes_end) ||
          i == back_curve && (j == 0 && end_matches_start ||
                              j == e_back_curve && ends_match)) {
        const auto intersections = curves_[i]->Intersect(*e.curves_[j]);
        const auto is_approximately_equal = [&start,
                                             &end](const bezier::Point &p) {
          return start.IsInNeighborhood(p) || end.IsInNeighborhood(p);
        };

        if (std::find_if_not(intersections.begin(), intersections.end(),
                             is_approximately_equal) != intersections.end()) {
          return true;
        };
      } else if (curves_[i]->IsIntersect(*e.curves_[j])) {
        return true;
      }
    }
  }

  return false;
}

bool Edge::IsStraightLine() const {
  const auto &points = curves_.front()->get_points();
  const auto start = points.front();
  const utils::Vector main_direction(points[1] - start);

  for (std::size_t i = 2; i < points.size(); i++) {
    const utils::Vector direction(points[i] - start);
    if (!direction.CollinearTo(main_direction)) {
      return false;
    }
  }

  for (std::size_t i = 1; i < curves_.size(); i++) {
    for (const auto &p : curves_[i]->get_points()) {
      const utils::Vector direction(p - start);
      if (!direction.CollinearTo(main_direction)) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace graph
