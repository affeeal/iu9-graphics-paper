#include "point.hpp"

namespace bezier {

bool Point::operator==(const Point &other) const {
  return x_ == other.x_ && y_ == other.y_;
}

Point Point::operator+(const Point &other) const {
  return Point(x_ + other.x_, y_ + other.y_);
}

Point Point::operator*(const double number) const {
  return Point(x_ * number, y_ * number);
}

} // namespace bezier
