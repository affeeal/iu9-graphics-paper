#include "rectangle.hpp"

#include <stdexcept>

namespace bezier {

Rectangle::Rectangle(const Point &top_left, const Point &bottom_right)
    : top_left_(top_left), bottom_right_(bottom_right) {
  CheckPointsMutualPositionInvariant();
}

const Point &Rectangle::get_top_left() const &noexcept { return top_left_; }

const Point &Rectangle::get_bottom_right() const &noexcept {
  return bottom_right_;
}

double Rectangle::Perimeter() const noexcept {
  return 2 * (bottom_right_.x - top_left_.x + top_left_.y - bottom_right_.y);
}

bool Rectangle::IsOverlap(const Rectangle &r) const noexcept {
  return top_left_.y >= r.bottom_right_.y && bottom_right_.y <= r.top_left_.y &&
         top_left_.x <= r.bottom_right_.x && bottom_right_.x >= r.top_left_.x;
}

Point Rectangle::Center() const noexcept {
  return top_left_.CenterWith(bottom_right_);
}

void Rectangle::CheckPointsMutualPositionInvariant() const {
  if (top_left_.y < bottom_right_.y || top_left_.x > bottom_right_.x) {
    throw std::logic_error(
        "Rectangle points mutual position invariant violated");
  }
}

}  // namespace bezier
