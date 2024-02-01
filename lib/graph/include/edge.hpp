#pragma once

#include <memory>
#include <vector>

#include "curve.hpp"
#include "vertex.hpp"

namespace graph {

class IEdge {
public:
  virtual bool IsIntersect(const IEdge &other) const = 0;

  virtual const IVertex &GetStart() const = 0;
  virtual const IVertex &GetEnd() const = 0;
  virtual const bezier::Curves &GetCurves() const = 0;

  virtual ~IEdge() {}
};

using IEdgeUptr = std::unique_ptr<IEdge>;

class Edge : public IEdge {
public:
  Edge() = delete;
  Edge(const IVertex &start, const IVertex &end, bezier::Curves &&curves)
      : start_(start), end_(end), curves_(std::move(curves)) {}

  bool IsIntersect(const IEdge &other) const override;

  const IVertex &GetStart() const override { return start_; }
  const IVertex &GetEnd() const override { return end_; }
  const bezier::Curves &GetCurves() const override { return curves_; }

private:
  const IVertex &start_;
  const IVertex &end_;
  bezier::Curves curves_;
};

} // namespace graph
