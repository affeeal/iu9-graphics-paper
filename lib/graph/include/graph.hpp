#pragma once

#include <functional>
#include <unordered_set>

#include "edge.hpp"

namespace graph {

namespace {

using ACPredicat = std::function<bool(const double angle)>;

enum class WriteIntersections {
  kSymmetrically,
  kAsymmetrically,
};

}  // namespace

class Graph;
class KLGrid;

using GraphUptr = std::unique_ptr<Graph>;

class Graph {
 public:
  enum class Filetype {
    kDot,
    kTex,
  };

  Graph() = delete;
  Graph(std::vector<VertexSptr> &&vertices, std::vector<EdgeSptr> &&edges)
      : vertices_(std::move(vertices)), edges_(std::move(edges)) {}

  // TODO: copy constructor, copy operator

  bool operator==(const Graph &other) const;

  static GraphUptr FromFile(const std::string &path,
                            const Filetype type = Filetype::kDot);

  bool IsStraightLine() const;

  std::vector<EdgeSptrConst> CheckKPlanar(const std::size_t k) const;

  std::vector<std::vector<EdgeSptrConst>> CheckKQuasiPlanar(
      const std::size_t k) const;

  std::vector<std::vector<EdgeSptrConst>> CheckKSkewness(
      const std::size_t k) const;

  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckRAC() const;

  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckACE(
      const double alpha) const;

  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckACL(
      const double alpha) const;

  std::vector<KLGrid> CheckGridFree(const std::size_t k,
                                    const std::size_t l) const;

  const std::vector<VertexSptr> &GetVertices() const { return vertices_; }
  const std::vector<EdgeSptr> &GetEdges() const { return edges_; }

 private:
  template <typename Set = std::unordered_set<std::size_t>>
  std::vector<Set> CalculateIntersections(
      const WriteIntersections mode = WriteIntersections::kSymmetrically) const;
  template <typename Container = std::unordered_set<std::size_t>>
  std::vector<EdgeSptrConst> EdgesByIndices(const Container &indices) const;
  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckAC(
      const ACPredicat &is_satisfying_angle) const;

  std::vector<VertexSptr> vertices_;
  std::vector<EdgeSptr> edges_;
};

struct KLGrid {
  std::vector<EdgeSptrConst> k_group;
  std::vector<EdgeSptrConst> l_group;

  KLGrid(const std::vector<EdgeSptrConst> &k_group,
         const std::vector<EdgeSptrConst> &l_group)
      : k_group(k_group), l_group(l_group) {}
};

}  // namespace graph
