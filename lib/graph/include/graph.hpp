#pragma once

#include <functional>
#include <unordered_set>

#include "edge.hpp"

namespace graph {

class Graph;
class KLGrid;

using GraphUptr = std::unique_ptr<Graph>;

class Graph final {
 public:
  Graph(const std::vector<Vertex> &vs);
  Graph(std::vector<Vertex> &&vs);
  Graph(std::vector<VertexSptrConst> &&vs, std::vector<EdgeSptrConst> &&es);

  // TODO: copy and move constructors, operators

  const std::vector<VertexSptrConst> &get_vertices() const &noexcept;
  const std::vector<EdgeSptrConst> &get_edges() const &noexcept;

  static GraphUptr Graphviz(const std::string &path);

  void AddVertex(const Vertex &v);
  void AddVertex(Vertex &&v);

  void AddVertices(const std::vector<Vertex> &vs);
  void AddVertices(std::vector<Vertex> &&vs);

  void AddSLEdge(const std::size_t start, const std::size_t end);
  void AddSLEdges(
      const std::vector<std::pair<std::size_t, std::size_t>> &vertex_pairs);

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

 private:
  using ACPredicat = std::function<bool(const double angle)>;

  enum class WriteIntersections {
    kSymmetrically,
    kAsymmetrically,
  };

  template <typename Set = std::unordered_set<std::size_t>>
  std::vector<Set> CalculateIntersections(
      const WriteIntersections mode = WriteIntersections::kSymmetrically) const;

  template <typename Container = std::unordered_set<std::size_t>>
  std::vector<EdgeSptrConst> EdgesByIndices(const Container &indices) const;

  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckAC(
      const ACPredicat &is_satisfying_angle) const;

  std::vector<VertexSptrConst> vs_;
  std::vector<EdgeSptrConst> es_;
};

struct KLGrid {
  std::vector<EdgeSptrConst> k_group;
  std::vector<EdgeSptrConst> l_group;

  KLGrid(const std::vector<EdgeSptrConst> &k_group,
         const std::vector<EdgeSptrConst> &l_group)
      : k_group(k_group), l_group(l_group) {}
};

}  // namespace graph
