#pragma once

#include <memory>
#include <vector>

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class Edge {
public:
  Edge() = delete;
  Edge(const Vertex &start, const Vertex &end, bezier::Curves &&curves)
      : start_(start), end_(end), curves_(std::move(curves)) {}

  bool IsIntersect(const Edge &other) const;

  bool operator==(const Edge &other) const;

  const Vertex &GetStart() const { return start_; }
  const Vertex &GetEnd() const { return end_; }
  const bezier::Curves &GetCurves() const { return curves_; }

private:
  const Vertex &start_;
  const Vertex &end_;
  bezier::Curves curves_;
};

} // namespace graph
