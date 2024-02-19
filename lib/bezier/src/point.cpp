#include "point.hpp"

#include <iostream>

namespace bezier {

Point::Point(const double x, const double y) : x_(x), y_(y) {}

bool Point::operator==(const Point &other) const {
  return x_ == other.x_ && y_ == other.y_;
}

Point Point::operator+(const Point &other) const {
  return {x_ + other.x_, y_ + other.y_};
}

Point Point::operator-(const Point &other) const {
  return {x_ - other.x_, y_ - other.y_};
}

Point Point::operator*(const double number) const {
  return {x_ * number, y_ * number};
}

std::ostream &operator<<(std::ostream &os, const Point &p) {
  std::cout << '{' << p.x_ << ", " << p.y_ << '}';
  return os;
}

Point Point::CenterWith(const Point &p) const {
  return {0.5 * (x_ + p.x_), 0.5 * (y_ + p.y_)};
}

bool Point::IsInNeighborhood(const Point &p, const double epsilon) const {
  return std::abs(x_ - p.x_) <= epsilon && std::abs(y_ - p.y_) <= epsilon;
}

}  // namespace bezier
