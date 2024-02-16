#include "rectangle.hpp"

#include <cassert>
#include <iostream>

namespace bezier {

Rectangle::Rectangle(Point top_left, Point bottom_right)
    : top_left_(std::move(top_left)), bottom_right_(std::move(bottom_right)) {
  assert(top_left_.GetY() >= bottom_right_.GetY() && bottom_right_.GetY() >= 0);
  assert(bottom_right_.GetX() >= top_left_.GetX() && top_left_.GetX() >= 0);
}

double Rectangle::CalculateArea() const {
  return (bottom_right_.GetX() - top_left_.GetX()) *
         (top_left_.GetY() - bottom_right_.GetY());
}

double Rectangle::CalculatePerimeter() const {
  const auto perimeter =  2 * (bottom_right_.GetX() - top_left_.GetX() + top_left_.GetY() -
              bottom_right_.GetY());
  std::cout << "perimeter: " << perimeter << std::endl;
  return perimeter;
}

bool Rectangle::IsOverlap(const Rectangle &other) const {
  return top_left_.GetY() >= other.GetBottomRight().GetY() &&
         bottom_right_.GetY() <= other.GetTopLeft().GetY() &&
         top_left_.GetX() <= other.GetBottomRight().GetX() &&
         bottom_right_.GetX() >= other.GetTopLeft().GetX();
}

Point Rectangle::CalculateCenter() const {
  return top_left_.CalculateCenter(bottom_right_);
}

}  // namespace bezier
