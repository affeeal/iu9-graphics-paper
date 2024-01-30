#include "curve.hpp"

#include <cassert>
#include <limits>
#include <type_traits>

#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

static constexpr auto kCurveCenterT = 0.5;

Curve::Curve(std::vector<Point> &&points) : points_(std::move(points)) {
  assert(points_.size() >= 2);
}

IRectangleUptr Curve::CalculateBoundingBox() const {
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
  std::vector<Point> left_curve_points, right_curve_points;
  left_curve_points.reserve(points_.size());
  right_curve_points.reserve(points_.size());

  SplitDeCasteljau(left_curve_points, right_curve_points, points_, t);

  assert(left_curve_points.size() == points_.size());
  assert(right_curve_points.size() == points_.size());

  return std::make_pair(std::make_unique<Curve>(std::move(left_curve_points)),
                        std::make_unique<Curve>(std::move(right_curve_points)));
}

void Curve::SplitDeCasteljau(std::vector<Point> &left_curve_points,
                             std::vector<Point> &right_curve_points,
                             const std::vector<Point> &points,
                             const double t) const {
  if (points.size() == 1) {
    left_curve_points.push_back(points.front());
    right_curve_points.push_back(points.front());
    return;
  }

  const std::size_t kNewSize = points.size() - 1;
  std::vector<Point> new_points;
  new_points.reserve(kNewSize);
  for (auto i = 0; i < kNewSize; i++) {
    if (i == 0) {
      left_curve_points.push_back(points[i]);
    }

    if (i == kNewSize - 1) {
      right_curve_points.push_back(points[i]);
    }

    new_points.push_back(points[i] * (1 - t) + points[i + 1] * t);
  }

  SplitDeCasteljau(left_curve_points, right_curve_points, new_points, t);
}

bool Curve::IsIntersect(const ICurve &other, const double threshold) const {
  return AreIntersect(*this, other, threshold);
}

bool Curve::AreIntersect(const ICurve& first, const ICurve& second,
                         const double threshold) const {
  const auto first_box = first.CalculateBoundingBox();
  const auto second_box = second.CalculateBoundingBox();

  assert(first_box);
  assert(second_box);

  if (!first_box->IsOverlap(*second_box)) {
    return false;
  }

  // One of the possible completion conditions
  if (first_box->CalculateArea() + second_box->CalculateArea() <  threshold) {
    return true;
  }

  const auto first_split = first.Split(kCurveCenterT);
  const auto second_split = second.Split(kCurveCenterT);

  assert(first_split.first);
  assert(first_split.second);
  assert(second_split.first);
  assert(second_split.second);

  return AreIntersect(*first_split.first, *second_split.first, threshold)
      || AreIntersect(*first_split.first, *second_split.second, threshold)
      || AreIntersect(*first_split.second, *second_split.first, threshold)
      || AreIntersect(*first_split.second, *second_split.second, threshold);
}

} // namespace bezier
