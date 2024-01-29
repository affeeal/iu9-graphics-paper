#pragma once

#include <utility>

namespace bezier {

struct Point {
  double x, y;

  explicit Point() : Point(0, 0) {}
  explicit Point(double x, double y) : x(std::move(x)), y(std::move(y)) {}
};

} // namespace bezier
