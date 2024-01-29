#include "curve.hpp"

#include <limits>

#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

IRectangleUptr Curve::CalculateBoundingRectangle() const {
  auto leftmost = std::numeric_limits<double>::max();
  auto topmost = std::numeric_limits<double>::min();
  auto rightmost = std::numeric_limits<double>::min();
  auto bottommost = std::numeric_limits<double>::max();

  for (const auto &point : points_) {
    if (point.x < leftmost) {
      leftmost = point.x;
    }

    if (point.y > topmost) {
      topmost = point.y;
    }

    if (point.x > rightmost) {
      rightmost = point.x;
    }

    if (point.y < bottommost) {
      bottommost = point.y;
    }
  }

  return std::make_unique<Rectangle>(Point(leftmost, topmost),
                                     Point(rightmost, bottommost));
}

std::pair<ICurveUptr, ICurveUptr> Curve::Split(const double t) const {
  return {}; // TODO
}

bool Curve::IsIntersect(const ICurve &other, const double threshold) const {
  return false; // TODO
}

} // namespace bezier
