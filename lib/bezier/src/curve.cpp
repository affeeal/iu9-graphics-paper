#include "curve.hpp"

#include <algorithm>
#include <iostream>
#include <limits>

#include "rectangle.hpp"

namespace bezier {

static constexpr auto kCurveCenterT = 0.5;

Curve::Curve(std::vector<Point> &&points) : points_(std::move(points)) {
  if (points_.size() < 2) {
    throw std::logic_error("A curve must consist of at least two points");
  }
}

bool Curve::operator==(const Curve &other) const {
  return points_ == other.points_;
}

std::ostream &operator<<(std::ostream &os, const Curve &c) {
  std::cout << '[';
  for (std::size_t i = 0; i < c.points_.size() - 1; i++) {
    std::cout << c.points_[i] << ", ";
  }
  std::cout << c.points_.back() << ']';

  return os;
}

RectangleUptr Curve::BoundingBox() const {
  auto leftmost_x = std::numeric_limits<double>::max();
  auto topmost_y = std::numeric_limits<double>::min();
  auto rightmost_x = std::numeric_limits<double>::min();
  auto bottommost_y = std::numeric_limits<double>::max();

  for (const auto &point : points_) {
    if (point.get_x() < leftmost_x) {
      leftmost_x = point.get_x();
    }

    if (point.get_y() > topmost_y) {
      topmost_y = point.get_y();
    }

    if (point.get_x() > rightmost_x) {
      rightmost_x = point.get_x();
    }

    if (point.get_y() < bottommost_y) {
      bottommost_y = point.get_y();
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

  first_curve_points.push_back(points.front());
  second_curve_points.push_back(points.back());

  for (std::size_t i = 0; i < new_size; i++) {
    new_points.push_back(points[i] * (1 - t) + points[i + 1] * t);
  }

  SplitDeCasteljau(first_curve_points, second_curve_points, new_points, t);
}

bool Curve::IsIntersect(const Curve &other, const double threshold) const {
  auto is_intersection_found = false;
  CheckIntersection(is_intersection_found, *this, other, threshold);
  return is_intersection_found;
}

// Optimized to find just one intersection
void Curve::CheckIntersection(bool &intersection_found, const Curve &first,
                              const Curve &second,
                              const double threshold) const {
  if (intersection_found) {
    return;
  }

  const auto first_box = first.BoundingBox();
  const auto second_box = second.BoundingBox();

  if (!first_box->IsOverlap(*second_box)) {
    return;
  }

  if (CompletionMetric(*first_box, *second_box) < threshold) {
    intersection_found = true;
    return;
  }

  const auto first_split = first.Split(kCurveCenterT);
  const auto second_split = second.Split(kCurveCenterT);

  CheckIntersection(intersection_found, *first_split.first, *second_split.first,
                    threshold);
  CheckIntersection(intersection_found, *first_split.first,
                    *second_split.second, threshold);
  CheckIntersection(intersection_found, *first_split.second,
                    *second_split.first, threshold);
  CheckIntersection(intersection_found, *first_split.second,
                    *second_split.second, threshold);
}

std::vector<Point> Curve::Intersect(const Curve &other,
                                    const double threshold) const {
  std::vector<Point> intersections;
  Intersect(intersections, *this, other, threshold);
  return intersections;
}

void Curve::Intersect(std::vector<Point> &intersections, const Curve &first,
                      const Curve &second, const double threshold) const {
  const auto first_box = first.BoundingBox();
  const auto second_box = second.BoundingBox();

  if (!first_box->IsOverlap(*second_box)) {
    return;
  }

  if (CompletionMetric(*first_box, *second_box) < threshold) {
    // One of the possible intersection approximation
    auto intersection = first_box->Center().CenterWith(second_box->Center());

    const auto is_approximately_equal = [&intersection](const Point &p) {
      return intersection.IsInNeighborhood(p);
    };

    if (std::find_if(intersections.begin(), intersections.end(),
                     is_approximately_equal) == intersections.end()) {
      intersections.push_back(std::move(intersection));
    }

    return;
  }

  const auto first_split = first.Split(kCurveCenterT);
  const auto second_split = second.Split(kCurveCenterT);

  Intersect(intersections, *first_split.first, *second_split.first, threshold);
  Intersect(intersections, *first_split.first, *second_split.second, threshold);
  Intersect(intersections, *first_split.second, *second_split.first, threshold);
  Intersect(intersections, *first_split.second, *second_split.second,
            threshold);
}

double Curve::CompletionMetric(const Rectangle &r1, const Rectangle &r2) const {
  return r1.Perimeter() + r2.Perimeter();
}

}  // namespace bezier
