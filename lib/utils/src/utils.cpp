#include "utils.hpp"

#include <cassert>
#include <cmath>

namespace utils {

std::size_t Combinations(const std::size_t n, const std::size_t k) {
  return Factorial(n) / Factorial(k) / Factorial(n - k);
}

Vector::Vector(const graph::Edge &edge) {
  assert(edge.get_curves().size() == 1);

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

std::string GetFileDirectory(const std::string &filename) {
  int end = filename.size() - 1;
  while (end >= 0 && filename[end] != '/') {
    end--;
  }

  return (end < 0) ? "./" : filename.substr(0, end + 1);
}

std::size_t Factorial(const std::size_t n) {
  return (n == 1 || n == 0) ? 1 : n * Factorial(n - 1);
}

}  // namespace utils
