#pragma once

#include <functional>
#include <unordered_set>

#include "edge.hpp"

namespace graph {

namespace {

using Edges = std::unordered_set<EdgeSptrConst>;
using EdgeIndices = std::unordered_set<std::size_t>;
using ACPredicat = std::function<bool(const double other_angle)>;

enum class IntersectionsPuttingDown {
  kSymmetric,
  kNonSymmetric,
};

class DirectionVector {
 public:
  DirectionVector() = delete;
  explicit DirectionVector(const Edge &edge);

  double CalculateAngle(const DirectionVector &other) const;
  double CalculateScalarProduct(const DirectionVector &other) const;
  double CalculateNorm() const;

  double GetX() const { return x_; }
  double GetY() const { return y_; }

 private:
  double x_;
  double y_;
};

}  // namespace

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

  /**
   * Check if the drawing belongs to the k-planar class.
   *
   * A k-planar drawing does not contain an edge crossed more than k times.
   *
   * @return edges crossed k or more times.
   */
  Edges CheckKPlanar(const std::size_t k) const;

  /**
   * Check if the drawing belongs to the k-quasi-planar class.
   *
   * A k-quasi planar drawing does not have k mutually crossing edges.
   *
   * @return edges mutually crossed k or more times.
   */
  Edges CheckKQuasiPlanar(const std::size_t k) const;

  /**
   * Check if the drawing belongs to the skewness-k class.
   *
   * A skewness-k drawing is such that the removal of at most k edges makes the
   * drawing planar.
   *
   * @return edges to remove if drawing is not skewness-k and empty set
   * otherwise.
   */
  Edges CheckKSkewness(const std::size_t k) const;

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
  Edges CheckRAC() const;

  /**
   * Check if the drawing belongs to the ACE-alpha class.
   *
   * In a straight-line ACE-alpha drawing any two crossing edges form an angle
   * equal to alpha.
   *
   * @param alpha An angle in the range [0, pi / 2].
   * @return edges that intersect at a different angle.
   */
  Edges CheckACE(const double alpha) const;

  /**
   * Check if the drawing belongs to the ACL-alpha class.
   *
   * In a straight-line ACL-alpha drawing the value of any crossing angle is at
   * least alpha.
   *
   * @param alpha An angle in the range [0, pi / 2].
   * @return edges that intersect at an angle less than alpha.
   */
  Edges CheckACL(const double alpha) const;

  const std::vector<VertexSptr> &GetVertices() const { return vertices_; }
  const std::vector<EdgeSptr> &GetEdges() const { return edges_; }

 private:
  Edges CheckAC(const ACPredicat &is_satisfying_angle) const;
  Edges GetEdgesByIndices(const EdgeIndices &indices) const;
  std::vector<EdgeIndices> CalculateIntersections(
      const IntersectionsPuttingDown mode =
          IntersectionsPuttingDown::kSymmetric) const;

  std::vector<VertexSptr> vertices_;
  std::vector<EdgeSptr> edges_;
};

}  // namespace graph
