#include "point.hpp"

#include <iostream>

namespace bezier {

Point::Point(const double x, const double y) noexcept : x(x), y(y) {}

bool Point::operator==(const Point &rhs) const noexcept {
  return x == rhs.x && y == rhs.y;
}

Point Point::operator+(const Point &rhs) const noexcept {
  return {x + rhs.x, y + rhs.y};
}

Point Point::operator-(const Point &rhs) const noexcept {
  return {x - rhs.x, y - rhs.y};
}

Point Point::operator*(const double rhs) const noexcept {
  return {x * rhs, y * rhs};
}

Point Point::CenterWith(const Point &p) const noexcept {
  return {0.5 * (x + p.x), 0.5 * (y + p.y)};
}

bool Point::IsInNeighborhood(const Point &p, const double eps) const noexcept {
  return std::abs(x - p.x) <= eps && std::abs(y - p.y) <= eps;
}

void Point::Dump(std::ostream &os) const { os << '{' << x << ", " << y << '}'; }

std::ostream &operator<<(std::ostream &os, const Point &p) {
  p.Dump(os);
  return os;
}

}  // namespace bezier
