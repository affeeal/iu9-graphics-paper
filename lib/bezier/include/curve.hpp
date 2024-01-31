#pragma once

#include <memory>
#include <vector>

#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

extern const double kThreshold;

class ICurve;

using ICurveUptr = std::unique_ptr<ICurve>;
using Curves = std::vector<bezier::ICurveUptr>;

class ICurve {
public:
  virtual IRectangleUptr CalculateBoundingBox() const = 0;
  virtual std::pair<ICurveUptr, ICurveUptr> Split(const double t) const = 0;
  virtual bool IsIntersect(const ICurve &other,
                           const double threshold = kThreshold) const = 0;

  virtual const std::vector<Point> &GetPoints() const = 0;

  virtual ~ICurve(){};
};

class Curve : public ICurve {
public:
  Curve() = delete;
  explicit Curve(std::vector<Point> &&points);

  IRectangleUptr CalculateBoundingBox() const override;
  std::pair<ICurveUptr, ICurveUptr> Split(const double t) const override;
  bool IsIntersect(const ICurve &other,
                   const double threshold = kThreshold) const override;

  const std::vector<Point> &GetPoints() const override { return points_; }

private:
  void SplitDeCasteljau(std::vector<Point> &left_curve_points,
                        std::vector<Point> &right_curve_points,
                        const std::vector<Point> &points, const double t) const;

  bool AreIntersect(const ICurve &first, const ICurve &second,
                    const double threshold) const;

  std::vector<Point> points_;
};

} // namespace bezier
