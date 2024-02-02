#pragma once

#include <vector>

#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

extern const double kThreshold;

class Curve;

using CurveUptr = std::unique_ptr<Curve>;

class Curve {
public:
  Curve() = delete;
  explicit Curve(std::vector<Point> &&points);

  RectangleUptr CalculateBoundingBox() const;
  std::pair<CurveUptr, CurveUptr> Split(const double t) const;
  bool IsIntersect(const Curve &other,
                   const double threshold = kThreshold) const;

  bool operator==(const Curve &other) const;

  const std::vector<Point> &GetPoints() const { return points_; }

private:
  void SplitDeCasteljau(std::vector<Point> &first_curve_points,
                        std::vector<Point> &second_curve_points,
                        const std::vector<Point> &points, const double t) const;

  bool AreIntersect(const Curve &first, const Curve &second,
                    const double threshold) const;

  std::vector<Point> points_;
};

} // namespace bezier
