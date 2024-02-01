#include "point.hpp"

#include <utility>

namespace bezier {

Point Point::operator+(const Point &other) const {
  return Point(x + other.x, y + other.y);
}

Point Point::operator*(const double number) const {
  return Point(x * number, y * number);
}

bool Point::operator==(const Point &other) const {
  return x == other.x && y == other.y;
}

} // namespace bezier
