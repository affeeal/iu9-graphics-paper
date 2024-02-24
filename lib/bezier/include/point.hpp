#pragma once

#include <ostream>

#include "utils.hpp"

namespace bezier {

struct Point {
  double x, y;

  bool operator==(const Point &rhs) const noexcept;
  Point operator+(const Point &rhs) const noexcept;
  Point operator-(const Point &rhs) const noexcept;
  Point operator*(const double rhs) const noexcept;

  Point CenterWith(const Point &p) const noexcept;
  bool IsInNeighborhood(const Point &p,
                        const double eps = utils::kEps) const noexcept;
  void Dump(std::ostream &os) const;
};

std::ostream &operator<<(std::ostream &os, const Point &p);

}  // namespace bezier
