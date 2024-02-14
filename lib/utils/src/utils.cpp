#include "utils.hpp"

#include <cassert>
#include <cmath>

namespace utils {

DirectionVector::DirectionVector(const graph::Edge &edge) {
  assert(edge.GetCurves().size() == 1);

  const auto start = edge.GetStart();
  const auto end = edge.GetEnd();

  x_ = end->GetX() - start->GetX();
  y_ = end->GetY() - start->GetY();
}

double DirectionVector::CalculateAngle(const DirectionVector &other) const {
  return std::acos(std::abs(CalculateScalarProduct(other)) / CalculateNorm() /
                   other.CalculateNorm());
}

double DirectionVector::CalculateNorm() const {
  return std::sqrt(x_ * x_ + y_ * y_);
}

double DirectionVector::CalculateScalarProduct(
    const DirectionVector &other) const {
  return x_ * other.x_ + y_ * other.y_;
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
