#include "edge.hpp"

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

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
  return start_ == other.start_ && end_ == other.GetEnd() &&
         curves_ == other.GetCurves();
}

} // namespace graph
