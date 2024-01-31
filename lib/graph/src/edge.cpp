#include "edge.hpp"

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

bool Edge::IsIntersect(const IEdge &other) const {
  for (const auto &curve : curves_) {
    for (const auto &other_curve : other.GetCurves()) {
      if (curve->IsIntersect(*other_curve)) {
        return true;
      }
    }
  }

  return false;
}

} // namespace graph
