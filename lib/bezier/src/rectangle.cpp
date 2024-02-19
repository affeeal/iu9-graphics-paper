#include "rectangle.hpp"

#include <cassert>

namespace bezier {

Rectangle::Rectangle(Point top_left, Point bottom_right)
    : top_left_(std::move(top_left)), bottom_right_(std::move(bottom_right)) {
  assert(top_left_.get_y() >= bottom_right_.get_y() &&
         bottom_right_.get_y() >= 0);
  assert(bottom_right_.get_x() >= top_left_.get_x() && top_left_.get_x() >= 0);
}

double Rectangle::Area() const {
  return (bottom_right_.get_x() - top_left_.get_x()) *
         (top_left_.get_y() - bottom_right_.get_y());
}

double Rectangle::Perimeter() const {
  return 2 * (bottom_right_.get_x() - top_left_.get_x() + top_left_.get_y() -
              bottom_right_.get_y());
}

bool Rectangle::IsOverlap(const Rectangle &other) const {
  return top_left_.get_y() >= other.get_bottom_right().get_y() &&
         bottom_right_.get_y() <= other.get_top_left().get_y() &&
         top_left_.get_x() <= other.get_bottom_right().get_x() &&
         bottom_right_.get_x() >= other.get_top_left().get_x();
}

Point Rectangle::Center() const { return top_left_.CenterWith(bottom_right_); }

}  // namespace bezier
