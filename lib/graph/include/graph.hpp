#pragma once

#include <memory>
#include <string_view>

#include "edge.hpp"
#include "vertex.hpp"

namespace graph {

class Graph;

using GraphUptr = std::unique_ptr<Graph>;

class Graph {
public:
  Graph() = delete;
  Graph(std::vector<IVertexUptr> &&vertices, std::vector<IEdgeUptr> &&edges)
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  static GraphUptr FromDotFile(const std::string &filename);

  const std::vector<IVertexUptr> &GetVertices() const { return vertices_; }
  const std::vector<IEdgeUptr> &GetEdges() const { return edges_; }

private:
  std::vector<IVertexUptr> vertices_;
  std::vector<IEdgeUptr> edges_;
};

} // namespace graph
