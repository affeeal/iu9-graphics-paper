#include "utils.hpp"

#include <cassert>
#include <cmath>

namespace utils {

Vector::Vector(const bezier::Point &point)
    : x_(point.get_x()), y_(point.get_y()) {}

Vector::Vector(const graph::Edge &edge) {
  if (!edge.IsStraightLine()) {
    throw std::logic_error(
        "Cannot create direction vector for non straight-line edge");
  }

  const auto start = edge.get_start();
  const auto end = edge.get_end();

  x_ = end->get_x() - start->get_x();
  y_ = end->get_y() - start->get_y();
}

double Vector::AngleWith(const Vector &other) const {
  return std::acos(std::abs(ScalarProduct(other)) / Norm() / other.Norm());
}

double Vector::Norm() const { return std::sqrt(x_ * x_ + y_ * y_); }

double Vector::ScalarProduct(const Vector &other) const {
  return x_ * other.x_ + y_ * other.y_;
}

bool Vector::CollinearTo(const Vector &other) const {
  return (x_ * other.y_ == other.x_ * y_);
}

std::size_t Factorial(const std::size_t n) {
  return (n == 1 || n == 0) ? 1 : n * Factorial(n - 1);
}

std::size_t Combinations(const std::size_t n, const std::size_t k) {
  return Factorial(n) / Factorial(k) / Factorial(n - k);
}

}  // namespace utils
