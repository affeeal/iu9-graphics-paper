#pragma once

#include <memory>

#include "point.hpp"

namespace bezier {

class Rectangle;

using RectangleUptr = std::unique_ptr<Rectangle>;

class Rectangle {
 public:
  Rectangle() = delete;
  Rectangle(Point top_left, Point bottom_right);

  double CalculateArea() const;
  bool IsOverlap(const Rectangle &other) const;

  const Point &GetTopLeft() const { return top_left_; }
  const Point &GetBottomRight() const { return bottom_right_; }

 private:
  Point top_left_;
  Point bottom_right_;
};

}  // namespace bezier
