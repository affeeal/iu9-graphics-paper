#pragma once

#include "edge.hpp"

namespace utils {

struct Vector final {
  double x, y;

  Vector(const bezier::Point &p) noexcept;
  Vector(const graph::Edge &e);

  double AngleWith(const Vector &v) const;
  double Norm() const;
  double ScalarProduct(const Vector &v) const noexcept;
  bool CollinearTo(const Vector &v) const noexcept;
};

}  // namespace utils
