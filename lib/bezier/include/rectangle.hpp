#pragma once

#include <memory>

#include "point.hpp"

namespace bezier {

class Rectangle;

using RectangleUptr = std::unique_ptr<Rectangle>;

class Rectangle {
 public:
  Rectangle() = default;
  Rectangle(Point top_left, Point bottom_right);

  double Area() const;
  double Perimeter() const;
  bool IsOverlap(const Rectangle &other) const;
  Point Center() const;

  const Point &get_top_left() const { return top_left_; }
  const Point &get_bottom_right() const { return bottom_right_; }

 private:
  Point top_left_;
  Point bottom_right_;
};

}  // namespace bezier
