#pragma once

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge;

using EdgeSptr = std::shared_ptr<Edge>;
using EdgeSptrConst = std::shared_ptr<const Edge>;

class Edge {
 public:
  Edge() = default;
  Edge(VertexSptrConst start, VertexSptrConst end);
  Edge(VertexSptrConst start, VertexSptrConst end,
       std::vector<bezier::CurveUptr> &&curves);

  bool operator==(const Edge &other) const;
  friend std::ostream &operator<<(std::ostream &os, const Edge &e);

  const VertexSptrConst &get_start() const { return start_; }
  const VertexSptrConst &get_end() const { return end_; }
  const std::vector<bezier::CurveUptr> &get_curves() const { return curves_; }

  bool IsIntersect(const Edge &other) const;
  bool IsStraightLine() const;

 private:
  VertexSptrConst start_;
  VertexSptrConst end_;
  std::vector<bezier::CurveUptr> curves_;
};

}  // namespace graph
