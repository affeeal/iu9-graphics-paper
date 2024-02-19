#pragma once

#include <functional>
#include <unordered_set>

#include "edge.hpp"

namespace graph {

namespace {

using ACPredicat = std::function<bool(const double other_angle)>;

enum class IntersectionsPuttingDown {
  kSymmetric,
  kNonSymmetric,
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

  /**
   * Check if the drawing belongs to the k-planar class.
   *
   * A k-planar drawing does not contain an edge crossed more than k times.
   *
   * @return edges crossed k or more times.
   */
  std::vector<EdgeSptrConst> CheckKPlanar(const std::size_t k) const;

  /**
   * Check if the drawing belongs to the k-quasi-planar class.
   *
   * A k-quasi planar drawing does not have k mutually crossing edges.
   *
   * @return edges mutually crossed k or more times.
   */
  std::vector<std::vector<EdgeSptrConst>> CheckKQuasiPlanar(
      const std::size_t k) const;

  /**
   * Check if the drawing belongs to the skewness-k class.
   *
   * A skewness-k drawing is such that the removal of at most k edges makes the
   * drawing planar.
   *
   * @return edges to remove if drawing is not skewness-k and empty set
   * otherwise.
   */
  std::vector<std::vector<EdgeSptrConst>> CheckKSkewness(
      const std::size_t k) const;

  /**
   * Check if the drawing belongs to the RAC class.
   *
   * A straight-line drawing such that any two crossing edges form pi / 2 angles
   * at their crossing point is straight-line RAC (RAC stands for Right Angle
   * Crossing).
   *
   * @param alpha An angle in the range [0, pi / 2].
   * @return edges that intersect at a different angle.
   */
  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckRAC() const;

  /**
   * Check if the drawing belongs to the ACE-alpha class.
   *
   * In a straight-line ACE-alpha drawing any two crossing edges form an angle
   * equal to alpha.
   *
   * @param alpha An angle in the range [0, pi / 2].
   * @return edges that intersect at a different angle.
   */
  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckACE(
      const double alpha) const;

  /**
   * Check if the drawing belongs to the ACL-alpha class.
   *
   * In a straight-line ACL-alpha drawing the value of any crossing angle is at
   * least alpha.
   *
   * @param alpha An angle in the range [0, pi / 2].
   * @return edges that intersect at an angle less than alpha.
   */
  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> CheckACL(
      const double alpha) const;

  std::vector<KLGrid> CheckGridFree(const std::size_t k,
                                    const std::size_t l) const;

  const std::vector<VertexSptr> &GetVertices() const { return vertices_; }
  const std::vector<EdgeSptr> &GetEdges() const { return edges_; }

 private:
  template <typename Set = std::unordered_set<std::size_t>>
  std::vector<Set> CalculateIntersections(
      const IntersectionsPuttingDown mode =
          IntersectionsPuttingDown::kSymmetric) const;
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
