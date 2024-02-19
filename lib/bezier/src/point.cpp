#include "point.hpp"

#include <iostream>

namespace bezier {

bool Point::operator==(const Point &other) const {
  return x_ == other.x_ && y_ == other.y_;
}

Point Point::operator+(const Point &other) const {
  return Point(x_ + other.x_, y_ + other.y_);
}

Point Point::operator-(const Point &other) const {
  return Point(x_ - other.x_, y_ - other.y_);
}

Point Point::operator*(const double number) const {
  return Point(x_ * number, y_ * number);
}

std::ostream &operator<<(std::ostream &os, const Point &p) {
  std::cout << '(' << p.x_ << ", " << p.y_ << ')';
  return os;
}

Point Point::CalculateCenter(const Point &other) const {
  return Point(0.5 * (x_ + other.x_), 0.5 * (y_ + other.y_));
}

bool Point::IsInNeighborhood(const Point &other, const double epsilon) const {
  return std::abs(x_ - other.x_) <= epsilon &&
         std::abs(y_ - other.y_) <= epsilon;
}

}  // namespace bezier
