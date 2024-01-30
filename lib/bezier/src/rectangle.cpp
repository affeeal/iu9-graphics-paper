#include "rectangle.hpp"

#include <cassert>

namespace bezier {

Rectangle::Rectangle(Point &&top_left, Point &&bottom_right)
    : top_left_(std::move(top_left)), bottom_right_(std::move(bottom_right)) {
  assert(top_left_.y >= bottom_right_.y && bottom_right_.y >= 0);
  assert(bottom_right_.x >= top_left_.x && top_left_.x >= 0);
}

double Rectangle::CalculateArea() const {
  return (bottom_right_.x - top_left_.x) * (top_left_.y - bottom_right_.y);
}

bool Rectangle::IsOverlap(const IRectangle &other) const {
  return top_left_.y >= other.GetBottomRight().y &&
         bottom_right_.y <= other.GetTopLeft().y &&
         top_left_.x <= other.GetBottomRight().x &&
         bottom_right_.x >= other.GetTopLeft().x;
}

} // namespace bezier
