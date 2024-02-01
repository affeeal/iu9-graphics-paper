#pragma once

#include "utils.hpp"

namespace bezier {

struct Point {
  double x, y;

  Point() : Point(0, 0) {}
  Point(double x, double y) : x(std::move(x)), y(std::move(y)) {}

  Point operator+(const Point &other) const;
  Point operator*(const double number) const;

  bool operator==(const Point &other) const;
};

} // namespace bezier
