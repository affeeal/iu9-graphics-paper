#pragma once

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge;

using EdgeUptr = std::unique_ptr<Edge>;

class Edge {
public:
  Edge() = delete;
  Edge(const Vertex &start, const Vertex &end,
       std::vector<bezier::CurveUptr> &&curves);

  bool IsIntersect(const Edge &other) const;

  bool operator==(const Edge &other) const;

  const Vertex &GetStart() const { return start_; }
  const Vertex &GetEnd() const { return end_; }
  const std::vector<bezier::CurveUptr> &GetCurves() const { return curves_; }

private:
  const Vertex &start_;
  const Vertex &end_;
  std::vector<bezier::CurveUptr> curves_;
};

} // namespace graph
