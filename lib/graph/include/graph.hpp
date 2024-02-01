#pragma once

#include <memory>

#include "edge.hpp"
#include "vertex.hpp"

namespace graph {

class Graph;

using GraphUptr = std::unique_ptr<Graph>;

class Graph {
public:
  Graph() = delete;
  Graph(std::vector<VertexUptr> &&vertices, std::vector<Edge> &&edges)
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  static GraphUptr FromDotFile(const std::string &filename);

  const std::vector<VertexUptr> &GetVertices() const { return vertices_; }
  const std::vector<Edge> &GetEdges() const { return edges_; }

private:
  std::vector<VertexUptr> vertices_;
  std::vector<Edge> edges_;
};

} // namespace graph
