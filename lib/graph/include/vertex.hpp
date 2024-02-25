#pragma once

#include <memory>

#include "point.hpp"

namespace graph {

class Vertex;

using VertexSptrConst = std::shared_ptr<const Vertex>;

struct Vertex final : public bezier::Point {
  std::string label;

  Vertex(const double x, const double y, const std::string& label) noexcept;

  void Dump(std::ostream& os) const override;
};

}  // namespace graph
