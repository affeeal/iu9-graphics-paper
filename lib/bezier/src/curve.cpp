#include "curve.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

#include "rectangle.hpp"

namespace bezier {

static constexpr auto kCurveCenterT = 0.5;

Curve::Curve(std::vector<Point> &&points) : points_(std::move(points)) {
  assert(points_.size() >= 2);
}

RectangleUptr Curve::CalculateBoundingBox() const {
  auto leftmost_x = std::numeric_limits<double>::max();
  auto topmost_y = std::numeric_limits<double>::min();
  auto rightmost_x = std::numeric_limits<double>::min();
  auto bottommost_y = std::numeric_limits<double>::max();

  for (const auto &point : points_) {
    if (point.GetX() < leftmost_x) {
      leftmost_x = point.GetX();
    }

    if (point.GetY() > topmost_y) {
      topmost_y = point.GetY();
    }

    if (point.GetX() > rightmost_x) {
      rightmost_x = point.GetX();
    }

    if (point.GetY() < bottommost_y) {
      bottommost_y = point.GetY();
    }
  }

  return std::make_unique<Rectangle>(Point(leftmost_x, topmost_y),
                                     Point(rightmost_x, bottommost_y));
}

std::pair<CurveUptr, CurveUptr> Curve::Split(const double t) const {
  std::vector<Point> first_curve_points;
  first_curve_points.reserve(points_.size());

  std::vector<Point> second_curve_points;
  second_curve_points.reserve(points_.size());

  SplitDeCasteljau(first_curve_points, second_curve_points, points_, t);

  assert(first_curve_points.size() == points_.size());
  assert(second_curve_points.size() == points_.size());

  return std::make_pair(
      std::make_unique<Curve>(std::move(first_curve_points)),
      std::make_unique<Curve>(std::move(second_curve_points)));
}

void Curve::SplitDeCasteljau(std::vector<Point> &first_curve_points,
                             std::vector<Point> &second_curve_points,
                             const std::vector<Point> &points,
                             const double t) const {
  if (points.size() == 1) {
    first_curve_points.push_back(points.front());
    second_curve_points.push_back(points.front());
    return;
  }

  const auto new_size = points.size() - 1;
  std::vector<Point> new_points;
  new_points.reserve(new_size);

  for (auto i = 0; i < new_size; i++) {
    if (i == 0) {
      first_curve_points.push_back(points[i]);
    }

    if (i == new_size - 1) {
      second_curve_points.push_back(points[i + 1]);
    }

    new_points.push_back(points[i] * (1 - t) + points[i + 1] * t);
  }

  SplitDeCasteljau(first_curve_points, second_curve_points, new_points, t);
}

bool Curve::IsIntersect(const Curve &other, const double threshold) const {
  return AreIntersect(*this, other, threshold);
}

bool Curve::AreIntersect(const Curve &first, const Curve &second,
                         const double threshold) const {
  const auto first_box = first.CalculateBoundingBox();
  const auto second_box = second.CalculateBoundingBox();

  assert(first_box);
  assert(second_box);

  if (!first_box->IsOverlap(*second_box)) {
    return false;
  }

  if (GetCompletionMetric(*first_box, *second_box) < threshold) {
    return true;
  }

  const auto first_split = first.Split(kCurveCenterT);
  const auto second_split = second.Split(kCurveCenterT);

  assert(first_split.first);
  assert(first_split.second);
  assert(second_split.first);
  assert(second_split.second);

  return AreIntersect(*first_split.first, *second_split.first, threshold) ||
         AreIntersect(*first_split.first, *second_split.second, threshold) ||
         AreIntersect(*first_split.second, *second_split.first, threshold) ||
         AreIntersect(*first_split.second, *second_split.second, threshold);
}

std::vector<Point> Curve::Intersect(const Curve &other,
                                    const double threshold) const {
  std::vector<Point> intersection_points;
  Intersect(intersection_points, *this, other, threshold);
  return intersection_points;
}

void Curve::Intersect(std::vector<Point> &intersection_points,
                      const Curve &first, const Curve &second,
                      const double threshold) const {
  std::cout << "intersecting curves " << first << " and " << second
            << std::endl;
  const auto first_box = first.CalculateBoundingBox();
  const auto second_box = second.CalculateBoundingBox();

  assert(first_box);
  assert(second_box);

  if (!first_box->IsOverlap(*second_box)) {
    return;
  }

  const auto metric = GetCompletionMetric(*first_box, *second_box);
  std::cout << "comparing " << metric << ", " << threshold << ": "
            << (metric < threshold) << std::endl;
  if (metric < threshold) {
    // One of the possible intersection approximation
    auto intersection_point = first_box->CalculateCenter().CalculateCenter(
        second_box->CalculateCenter());

    auto is_approximately_equal = [&intersection_point](const Point &other) {
      return intersection_point.IsInNeighborhood(other);
    };

    if (std::find_if(intersection_points.begin(), intersection_points.end(),
                     is_approximately_equal) == intersection_points.end()) {
      intersection_points.push_back(std::move(intersection_point));
    }

    return;
  }

  const auto first_split = first.Split(kCurveCenterT);
  const auto second_split = second.Split(kCurveCenterT);

  assert(first_split.first);
  assert(first_split.second);
  assert(second_split.first);
  assert(second_split.second);

  Intersect(intersection_points, *first_split.first, *second_split.first,
            threshold);
  Intersect(intersection_points, *first_split.first, *second_split.second,
            threshold);
  Intersect(intersection_points, *first_split.second, *second_split.first,
            threshold);
  Intersect(intersection_points, *first_split.second, *second_split.second,
            threshold);
}

bool Curve::operator==(const Curve &other) const {
  return points_ == other.points_;
}

std::ostream &operator<<(std::ostream &os, const Curve &curve) {
  for (std::size_t i = 0; i < curve.points_.size(); i++) {
    std::cout << curve.points_[i];

    if (i + 1 < curve.points_.size()) {
      std::cout << '-';
    }
  }

  return os;
}

double Curve::GetCompletionMetric(const Rectangle &r1,
                                  const Rectangle &r2) const {
  return r1.CalculatePerimeter() + r2.CalculatePerimeter();
}

}  // namespace bezier
