#pragma once

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge;

using EdgeSptr = std::shared_ptr<Edge>;
using EdgeSptrConst = std::shared_ptr<const Edge>;

class Edge {
 public:
  Edge() = delete;
  Edge(VertexSptrConst start, VertexSptrConst end,
       std::vector<bezier::CurveUptr> &&curves);

  bool IsIntersect(const Edge &other) const;
  bool IsStraightLine() const;

  bool operator==(const Edge &other) const;
  friend std::ostream &operator<<(std::ostream &os, const Edge &edge);

  VertexSptrConst GetStart() const { return start_; }
  VertexSptrConst GetEnd() const { return end_; }
  const std::vector<bezier::CurveUptr> &GetCurves() const { return curves_; }

 private:
  VertexSptrConst start_;
  VertexSptrConst end_;
  std::vector<bezier::CurveUptr> curves_;
};

}  // namespace graph
