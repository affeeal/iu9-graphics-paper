#pragma once

#include <vector>

#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

constexpr double kThreshold = 10e-4;

class Curve;

using CurveUptr = std::unique_ptr<Curve>;

class Curve {
 public:
  Curve() = default;
  explicit Curve(std::vector<Point> &&points);

  bool operator==(const Curve &other) const;
  friend std::ostream &operator<<(std::ostream &os, const Curve &curve);

  const std::vector<Point> &get_points() const { return points_; }

  RectangleUptr BoundingBox() const;
  std::pair<CurveUptr, CurveUptr> Split(const double t) const;
  bool IsIntersect(const Curve &other,
                   const double threshold = kThreshold) const;
  std::vector<Point> Intersect(const Curve &other,
                               const double threshould = kThreshold) const;

 private:
  void SplitDeCasteljau(std::vector<Point> &first_curve_points,
                        std::vector<Point> &second_curve_points,
                        const std::vector<Point> &points, const double t) const;
  void CheckIntersection(bool &is_found, const Curve &first,
                         const Curve &second, const double threshold) const;
  void Intersect(std::vector<Point> &intersection_points, const Curve &first,
                 const Curve &second, const double threshold) const;
  double CompletionMetric(const Rectangle &r1, const Rectangle &r2) const;

  std::vector<Point> points_;
};

}  // namespace bezier
