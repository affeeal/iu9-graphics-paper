#include "vertex.hpp"

#include <iostream>

namespace graph {

Vertex::Vertex(double x, double y, const std::string& label)
    : x_(std::move(x)), y_(std::move(y)), label_(label) {}

bool Vertex::operator==(const Vertex& other) const {
  return x_ == other.x_ && y_ == other.y_ && label_ == other.label_;
}

std::ostream& operator<<(std::ostream& os, const Vertex& vertex) {
  std::cout << '(' << vertex.label_ << ": " << vertex.x_ << ", " << vertex.y_
            << ')';
  return os;
}

bezier::Point Vertex::AsPoint() const { return bezier::Point(x_, y_); }

}  // namespace graph
