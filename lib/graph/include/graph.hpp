#pragma once

#include <unordered_set>

#include "edge.hpp"

namespace {

using EdgeIndices = std::unordered_set<std::size_t>;

}  // namespace

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

  std::unordered_set<EdgeSptrConst> CheckForKPlanarity(
      const std::size_t k) const;
  std::unordered_set<EdgeSptrConst> CheckForKQuasiPlanarity(
      const std::size_t k) const;
  std::unordered_set<EdgeSptrConst> CheckForKSkewness(
      const std::size_t k) const;

  const std::vector<VertexSptr> &GetVertices() const { return vertices_; }
  const std::vector<EdgeSptr> &GetEdges() const { return edges_; }

 private:
  std::vector<EdgeIndices> CalculateEdgeIntersections() const;
  std::unordered_set<EdgeSptrConst> GetEdgesByIndices(
      const EdgeIndices &indices) const;

  std::vector<VertexSptr> vertices_;
  std::vector<EdgeSptr> edges_;
};

}  // namespace graph
