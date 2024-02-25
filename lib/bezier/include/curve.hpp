#pragma once

#include <memory>
#include <vector>

#include "rectangle.hpp"

namespace bezier {

constexpr auto kDefaultThreshold = 1e-3;

class Curve;

using CurveUptrConst = std::unique_ptr<const Curve>;

class Curve final {
 public:
  Curve(const std::vector<Point> &ps);
  Curve(std::vector<Point> &&ps);

  bool operator==(const Curve &rhs) const;

  const std::vector<Point> &get_points() const &noexcept;

  Rectangle BoundingRectangle() const;
  std::pair<std::unique_ptr<Curve>, std::unique_ptr<Curve>> Split(
      const double t) const;
  bool IsIntersect(const Curve &c, const double eps = kDefaultThreshold) const;
  std::vector<Point> Intersect(const Curve &c,
                               const double eps = kDefaultThreshold) const;
  void Dump(std::ostream &os) const;

 private:
  void CheckPointsSizeInvariant() const;
  void SplitDeCasteljau(std::vector<Point> &ps1, std::vector<Point> &ps2,
                        const std::vector<Point> &ps, const double t) const;
  bool IsIntersect(const Curve &c1, const Curve &c2, const double eps) const;
  void Intersect(std::vector<Point> &ps, const Curve &c1, const Curve &c2,
                 const double eps) const;
  double CompletionMetric(const Rectangle &r1,
                          const Rectangle &r2) const noexcept;

  std::vector<Point> ps_;
};

std::ostream &operator<<(std::ostream &os, const Curve &rhs);

}  // namespace bezier
