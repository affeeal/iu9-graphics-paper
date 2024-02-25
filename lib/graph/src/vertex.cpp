#include "vertex.hpp"

namespace graph {

Vertex::Vertex(const double x, const double y,
               const std::string& label) noexcept
    : bezier::Point(x, y), label(label) {}

void Vertex::Dump(std::ostream& os) const {
  os << '{' << x << ", " << y << ", " << label << '}';
}

}  // namespace graph
