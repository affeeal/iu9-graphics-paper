#pragma once

#include <utility>

namespace bezier {

class Point {
public:
  Point(double x, double y) : x_(std::move(x)), y_(std::move(y)) {}

  bool operator==(const Point &other) const;
  Point operator+(const Point &other) const;
  Point operator*(const double other) const;

  double GetX() const { return x_; }
  double GetY() const { return y_; }

private:
  double x_;
  double y_;
};

} // namespace bezier
