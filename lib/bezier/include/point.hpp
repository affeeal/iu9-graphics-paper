#pragma once

#include <ostream>
#include <utility>

namespace bezier {

constexpr double kEpsilon = 10e-3;

class Point {
 public:
  Point() : x_(0), y_(0) {}
  Point(const double x, const double y) : x_(x), y_(y) {}

  bool operator==(const Point &other) const;
  Point operator+(const Point &other) const;
  Point operator*(const double other) const;
  friend std::ostream &operator<<(std::ostream &os, const Point &p);

  Point CalculateCenter(const Point &other) const;
  bool IsInNeighborhood(const Point &other,
                        const double epsilon = kEpsilon) const;

  double GetX() const { return x_; }
  double GetY() const { return y_; }

 private:
  double x_;
  double y_;
};

}  // namespace bezier
