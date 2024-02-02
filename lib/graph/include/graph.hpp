#pragma once

#include "edge.hpp"

namespace graph {

class Graph;

using GraphUptr = std::unique_ptr<Graph>;

class Graph {
public:
  Graph() = delete;
  Graph(std::vector<VertexUptr> &&vertices, std::vector<EdgeUptr> &&edges)
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  static GraphUptr FromDotFile(const std::string &filepath);

  const std::vector<VertexUptr> &GetVertices() const { return vertices_; }
  const std::vector<EdgeUptr> &GetEdges() const { return edges_; }

private:
  std::vector<VertexUptr> vertices_;
  std::vector<EdgeUptr> edges_;
};

} // namespace graph
