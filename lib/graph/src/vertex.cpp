#include "vertex.hpp"

#include <iostream>

namespace graph {

Vertex::Vertex(const double x, const double y) : x_(x), y_(y), label_() {}

Vertex::Vertex(const double x, const double y, const std::string& label)
    : x_(x), y_(y), label_(label) {}

bool Vertex::operator==(const Vertex& other) const {
  return x_ == other.x_ && y_ == other.y_ && label_ == other.label_;
}

std::ostream& operator<<(std::ostream& os, const Vertex& v) {
  std::cout << '(' << v.label_ << ": " << v.x_ << ", " << v.y_ << ')';
  return os;
}

bezier::Point Vertex::AsPoint() const { return bezier::Point(x_, y_); }

}  // namespace graph
