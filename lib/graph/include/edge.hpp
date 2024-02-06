#pragma once

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge;

using EdgeSptr = std::shared_ptr<Edge>;

class Edge {
public:
  Edge() = delete;
  Edge(VertexSptrConst start, VertexSptrConst end,
       std::vector<bezier::CurveUptr> &&curves);

  bool IsIntersect(const Edge &other) const;

  bool operator==(const Edge &other) const;

  VertexSptrConst GetStart() const { return start_; }
  VertexSptrConst GetEnd() const { return end_; }
  const std::vector<bezier::CurveUptr> &GetCurves() const { return curves_; }

private:
  VertexSptrConst start_;
  VertexSptrConst end_;
  std::vector<bezier::CurveUptr> curves_;
};

} // namespace graph
