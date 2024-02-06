#pragma once

#include "edge.hpp"

namespace graph {

class Graph;

using GraphUptr = std::unique_ptr<Graph>;

class Graph {
public:
  Graph() = delete;
  Graph(std::vector<VertexSptr> &&vertices, std::vector<EdgeSptr> &&edges)
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  // TODO: copy constructor, copy operator

  bool operator==(const Graph &other) const;

  static GraphUptr FromDotFile(const std::string &filepath);

  const std::vector<VertexSptr> &GetVertices() const { return vertices_; }
  const std::vector<EdgeSptr> &GetEdges() const { return edges_; }

private:
  std::vector<VertexSptr> vertices_;
  std::vector<EdgeSptr> edges_;
};

} // namespace graph
