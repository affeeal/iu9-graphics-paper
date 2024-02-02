#include "edge.hpp"

namespace graph {

Edge::Edge(const Vertex &start, const Vertex &end,
           std::vector<bezier::CurveUptr> &&curves)
    : start_(start), end_(end), curves_(std::move(curves)) {}

bool Edge::IsIntersect(const Edge &other) const {
  for (const auto &curve : curves_) {
    for (const auto &other_curve : other.GetCurves()) {
      if (curve->IsIntersect(*other_curve)) {
        return true;
      }
    }
  }

  return false;
}

bool Edge::operator==(const Edge &other) const {
  if (start_ != other.GetStart() || end_ != other.GetEnd() ||
      curves_.size() != other.GetCurves().size()) {
    return false;
  }

  const auto &other_curves = other.GetCurves();
  for (auto i = 0; i < curves_.size(); i++) {
    if (curves_[i]->GetPoints() != other_curves[i]->GetPoints()) {
      return false;
    }
  }

  return true;
}

} // namespace graph
