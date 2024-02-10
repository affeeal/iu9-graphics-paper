#include "edge.hpp"

namespace graph {

Edge::Edge(VertexSptrConst start, VertexSptrConst end,
           std::vector<bezier::CurveUptr> &&curves)
    : start_(std::move(start)),
      end_(std::move(end)),
      curves_(std::move(curves)) {}

bool Edge::IsIntersect(const Edge &other) const {
  for (const auto &curve : curves_) {
    for (const auto &other_curve : other.curves_) {
      if (curve->IsIntersect(*other_curve)) {
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

}  // namespace graph
