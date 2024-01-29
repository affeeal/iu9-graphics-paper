#pragma once

#include <memory>
#include <vector>

#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

class ICurve;

using ICurveUptr = std::unique_ptr<ICurve>;

class ICurve {
public:
  virtual IRectangleUptr CalculateBoundingRectangle() const = 0;
  virtual std::pair<ICurveUptr, ICurveUptr> Split(const double t) const = 0;
  virtual bool IsIntersect(const ICurve &other,
                           const double threshold) const = 0;

  virtual const std::vector<Point> &GetPoints() const = 0;

  virtual ~ICurve(){};
};

class Curve : public ICurve {
public:
  Curve() = delete;
  Curve(std::vector<Point> &&points) : points_(std::move(points)) {}

  IRectangleUptr CalculateBoundingRectangle() const override;
  std::pair<ICurveUptr, ICurveUptr> Split(const double t) const override;
  bool IsIntersect(const ICurve &other, const double threshold) const override;

  const std::vector<Point> &GetPoints() const override { return points_; }

private:
  std::vector<Point> points_;
};

} // namespace bezier
