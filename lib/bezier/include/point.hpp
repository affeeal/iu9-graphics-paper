#pragma once

#include <ostream>

namespace bezier {

constexpr double kEpsilon = 10e-4;

class Point {
 public:
  Point() = default;
  Point(const double x, const double y);

  bool operator==(const Point &other) const;
  Point operator+(const Point &other) const;
  Point operator-(const Point &other) const;
  Point operator*(const double other) const;
  friend std::ostream &operator<<(std::ostream &os, const Point &p);

  Point CenterWith(const Point &p) const;
  bool IsInNeighborhood(const Point &p, const double epsilon = kEpsilon) const;

  double get_x() const { return x_; }
  double get_y() const { return y_; }

 private:
  double x_;
  double y_;
};

}  // namespace bezier
