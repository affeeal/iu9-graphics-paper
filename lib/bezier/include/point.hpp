#pragma once

#include <ostream>

namespace bezier {

constexpr auto kDefaultNeighborhood = 1e-3;

struct Point {
  double x, y;

  Point(const double x, const double y) noexcept;

  bool operator==(const Point &rhs) const noexcept;
  Point operator+(const Point &rhs) const noexcept;
  Point operator-(const Point &rhs) const noexcept;
  Point operator*(const double rhs) const noexcept;

  Point CenterWith(const Point &p) const noexcept;
  bool IsInNeighborhood(const Point &p,
                        const double eps = kDefaultNeighborhood) const noexcept;
  virtual void Dump(std::ostream &os) const;
};

std::ostream &operator<<(std::ostream &os, const Point &p);

}  // namespace bezier
