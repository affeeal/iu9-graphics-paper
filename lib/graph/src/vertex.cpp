#include "vertex.hpp"

namespace graph {

Vertex::Vertex(double x, double y, const std::string &label)
    : x_(std::move(x)), y_(std::move(y)), label_(label) {}

bool Vertex::operator==(const Vertex &other) const {
  return x_ == other.x_ && y_ == other.y_ && label_ == other.label_;
}

}  // namespace graph
