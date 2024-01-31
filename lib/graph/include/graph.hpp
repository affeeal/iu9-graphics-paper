#pragma once

#include <memory>
#include <string_view>

#include "edge.hpp"
#include "vertex.hpp"

namespace graph {

class IGraph;

using IGraphUptr = std::unique_ptr<IGraph>;

class IGraph {
public:
  virtual const std::vector<IVertexUptr> &GetVertices() const = 0;
  virtual const std::vector<IEdgeUptr> &GetEdges() const = 0;

  virtual ~IGraph() {}
};

class Graph : public IGraph {
public:
  Graph() = delete;
  explicit Graph(std::vector<IVertexUptr> &&vertices,
                 std::vector<IEdgeUptr> &&edges)
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  static IGraphUptr FromDotFile(std::string_view filename);

  const std::vector<IVertexUptr> &GetVertices() const override {
    return vertices_;
  }
  const std::vector<IEdgeUptr> &GetEdges() const override { return edges_; }

private:
  std::vector<IVertexUptr> vertices_;
  std::vector<IEdgeUptr> edges_;
};

} // namespace graph
