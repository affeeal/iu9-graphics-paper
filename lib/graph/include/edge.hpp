#pragma once

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge;

using EdgeSptrConst = std::shared_ptr<const Edge>;

class Edge final {
 public:
  Edge(VertexSptrConst start, VertexSptrConst end);
  Edge(VertexSptrConst start, VertexSptrConst end,
       std::vector<bezier::CurveUptrConst> &&curves);

  const VertexSptrConst &get_start() const &noexcept;
  const VertexSptrConst &get_end() const &noexcept;
  const std::vector<bezier::CurveUptrConst> &get_curves() const &noexcept;

  bool IsIntersect(const Edge &e) const;
  bool IsStraightLine() const;
  void Dump(std::ostream& os) const;

 private:
  void CheckCurvesSizeInvariant() const;

  VertexSptrConst start_;
  VertexSptrConst end_;
  std::vector<bezier::CurveUptrConst> cs_;
};

std::ostream &operator<<(std::ostream &os, const Edge &e);

}  // namespace graph
