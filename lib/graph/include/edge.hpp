#pragma once

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge;

using EdgeSptr = std::shared_ptr<Edge>;

class Edge {
public:
  Edge() = delete;
  Edge(VertexSptr start, VertexSptr end,
       std::vector<bezier::CurveUptr> &&curves);

  bool IsIntersect(const Edge &other) const;

  bool operator==(const Edge &other) const;

  VertexSptr GetStart() const { return start_; }
  VertexSptr GetEnd() const { return end_; }
  const std::vector<bezier::CurveUptr> &GetCurves() const { return curves_; }

private:
  VertexSptr start_;
  VertexSptr end_;
  std::vector<bezier::CurveUptr> curves_;
};

} // namespace graph
