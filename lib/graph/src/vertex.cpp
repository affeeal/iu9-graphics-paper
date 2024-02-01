#include "vertex.hpp"

#include "utils.hpp"

namespace graph {

Vertex::Vertex(double x, double y, const std::string& label)
    : x_(std::move(x)), y_(std::move(y)), label_(label) {}

} // namespace graph
