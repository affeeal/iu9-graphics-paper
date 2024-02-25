#include "vector.hpp"

#include <cmath>

namespace utils {

Vector::Vector(const bezier::Point &p) noexcept : x(p.x), y(p.y) {}

Vector::Vector(const graph::Edge &e) {
  if (!e.IsStraightLine()) {
    throw std::logic_error("Cannot create non straight-line edge Vector2D");
  }

  const auto &start = e.get_start();
  const auto &end = e.get_end();

  x = end->x - start->x;
  y = end->y - start->y;
}

double Vector::AngleWith(const Vector &v) const {
  return std::acos(std::abs(ScalarProduct(v)) / Norm() / v.Norm());
}

double Vector::Norm() const { return std::sqrt(x * x + y * y); }

double Vector::ScalarProduct(const Vector &v) const noexcept {
  return x * v.x + y * v.y;
}

bool Vector::CollinearTo(const Vector &v) const noexcept {
  return (x * v.y == v.x * y);
}

}  // namespace utils
