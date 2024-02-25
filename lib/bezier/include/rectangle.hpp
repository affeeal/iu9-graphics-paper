#pragma once

#include "point.hpp"

namespace bezier {

class Rectangle;

class Rectangle final {
 public:
  Rectangle(const Point &top_left, const Point &bottom_right);

  const Point &get_top_left() const &noexcept;
  const Point &get_bottom_right() const &noexcept;

  double Perimeter() const noexcept;
  bool IsOverlap(const Rectangle &r) const noexcept;
  Point Center() const noexcept;

 private:
  void CheckPointsMutualPositionInvariant() const;

  Point top_left_;
  Point bottom_right_;
};

}  // namespace bezier
