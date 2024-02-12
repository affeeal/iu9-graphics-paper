#include "graph.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "edge.hpp"
#include "utils.hpp"

namespace graph {

namespace {

using LabelToVertex = std::unordered_map<std::string, VertexSptr>;

constexpr std::string_view kTikzpictureStart = "\\begin{tikzpicture}";
constexpr std::string_view kTikzpictureEnd = "\\end{tikzpicture}";
constexpr std::string_view kNodeCommandStart = "\\node";
constexpr std::string_view kDrawCommandStart = "\\draw";

constexpr char kNodeStart = '(';
constexpr char kNodeEnd = ')';
constexpr char kMeasurementUnitStart = 'b';
constexpr char kCoordinatesDivider = ',';
constexpr char kDrawCommandEnd = ';';

constexpr std::size_t kCurveSize = 4;

DirectionVector::DirectionVector(const Edge &edge) {
  assert(edge.GetCurves().size() == 1);

  const auto start = edge.GetStart();
  const auto end = edge.GetEnd();

  x_ = end->GetX() - start->GetX();
  y_ = end->GetY() - start->GetY();
}

double DirectionVector::CalculateAngle(const DirectionVector &other) const {
  return std::acos(std::abs(CalculateScalarProduct(other)) / CalculateNorm() /
                   other.CalculateNorm());
}

double DirectionVector::CalculateNorm() const {
  return std::sqrt(x_ * x_ + y_ * y_);
}

double DirectionVector::CalculateScalarProduct(
    const DirectionVector &other) const {
  return x_ * other.x_ + y_ * other.y_;
}

std::vector<std::string> GetNodes(std::string &&command) {
  std::vector<std::string> nodes;
  std::stringstream ss(command);

  while (true) {
    while (!ss.eof() && ss.get() != kNodeStart) {
    }

    if (ss.eof()) {
      break;
    }

    std::string node = "";
    while (!ss.eof()) {
      const auto character = ss.get();
      if (character == kNodeEnd) {
        break;
      }

      node.push_back(character);
    }

    if (ss.eof()) {
      throw std::runtime_error("Failed to parse command node");
    }

    nodes.push_back(std::move(node));
  }

  return nodes;
}

std::pair<double, double> NodeToCoordinates(std::string &&node) {
  std::stringstream ss(std::move(node));

  std::string x = "";
  while (!ss.eof()) {
    const auto character = ss.get();
    if (character == kMeasurementUnitStart) {
      break;
    }

    x.push_back(character);
  }

  while (!ss.eof() && ss.get() != kCoordinatesDivider) {
  }

  std::string y = "";
  while (!ss.eof()) {
    const auto character = ss.get();
    if (character == kMeasurementUnitStart) {
      break;
    }

    y.push_back(character);
  }

  if (ss.eof()) {
    throw std::runtime_error("Failed to parse node coordinates");
  }

  return std::make_pair(std::stod(std::move(x)), std::stod(std::move(y)));
}

void HandleNodeCommand(std::string &&command,
                       LabelToVertex &labels_to_vertices) {
  auto nodes = GetNodes(std::move(command));
  assert(nodes.size() == 2);  // label and appropriate coordinates

  const auto coordinates = NodeToCoordinates(std::move(nodes.back()));
  labels_to_vertices[nodes.front()] = std::make_shared<Vertex>(
      coordinates.first, coordinates.second, nodes.front());
}

std::vector<bezier::Point> NodesToPoints(
    std::vector<std::string> &&nodes, const LabelToVertex &labels_to_vertices) {
  std::vector<bezier::Point> points;
  points.reserve(nodes.size());

  for (auto i = 0; i < nodes.size(); i++) {
    if (i == 0 || i == nodes.size() - 1) {
      const auto &vertex = labels_to_vertices.at(std::move(nodes[i]));
      points.push_back(bezier::Point(vertex->GetX(), vertex->GetY()));
    } else {
      const auto coordinates = NodeToCoordinates(std::move(nodes[i]));
      points.push_back(bezier::Point(coordinates.first, coordinates.second));
    }
  }

  return points;
}

std::vector<bezier::CurveUptr> CurvesByPoints(
    std::vector<bezier::Point> &&points) {
  std::vector<bezier::CurveUptr> curves;
  curves.reserve((points.size() + 1) / kCurveSize);

  for (auto i = 0; i < curves.capacity(); i++) {
    std::vector<bezier::Point> curve_points;
    curve_points.reserve(kCurveSize);

    for (auto j = 0; j < kCurveSize - 1; j++) {
      curve_points.push_back(std::move(points[i * (kCurveSize - 1) + j]));
    }
    curve_points.push_back(points[(i + 1) * (kCurveSize - 1)]);

    curves.push_back(std::make_unique<bezier::Curve>(std::move(curve_points)));
  }

  return curves;
}

void HandleDrawCommand(std::string &&command, std::vector<EdgeSptr> &edges,
                       const LabelToVertex &labels_to_vertices) {
  auto nodes = GetNodes(std::move(command));

  assert(nodes.size() == kCurveSize ||
         nodes.size() > kCurveSize &&
             nodes.size() % kCurveSize == kCurveSize - 1);

  const auto &start = labels_to_vertices.at(nodes.front());
  const auto &end = labels_to_vertices.at(nodes.back());

  auto points = NodesToPoints(std::move(nodes), labels_to_vertices);
  auto curves = CurvesByPoints(std::move(points));

  edges.push_back(std::make_shared<Edge>(start, end, std::move(curves)));
}

DirectionVector GetDirectionVector(const Edge &edge) {
  return DirectionVector(end->GetX() - start->GetX(),
                         end->GetY() - start->GetY());
}

}  // namespace

// NOTE: can be optimized with sorting
bool Graph::operator==(const Graph &other) const {
  for (const auto &vertex : vertices_) {
    auto compare = [&vertex](const VertexSptr &other_vertex) {
      assert(vertex && other_vertex);
      return *vertex == *other_vertex;
    };

    if (std::find_if(other.vertices_.begin(), other.vertices_.end(),
                     std::move(compare)) == other.vertices_.end()) {
      return false;
    }
  }

  for (const auto &edge : edges_) {
    auto compare = [&edge](const EdgeSptr &other_edge) {
      return *edge == *other_edge;
    };

    if (std::find_if(other.edges_.begin(), other.edges_.end(),
                     std::move(compare)) == other.edges_.end()) {
      return false;
    }
  }

  return true;
}

GraphUptr Graph::FromDotFile(const std::string &filepath) {
  {
    const auto command =
        "dot2tex " + filepath + " -ftikz -tmath -o " + filepath + ".tex";
    std::system(command.c_str());
  }

  std::ifstream tex_file(filepath + ".tex");
  if (!tex_file.is_open()) {
    throw std::runtime_error("Failed to open .tex file");
  }

  std::string line = "";
  while (std::getline(tex_file, line)) {
    if (line.find(kTikzpictureStart) != std::string::npos) {
      break;
    }
  }

  if (tex_file.eof()) {
    throw std::runtime_error("Failed to find the tikzpicture start");
  }

  LabelToVertex labels_to_vertices;
  std::vector<EdgeSptr> edges;

  while (std::getline(tex_file, line)) {
    if (line.find(kNodeCommandStart) != std::string::npos) {
      HandleNodeCommand(std::move(line), labels_to_vertices);
    } else if (line.find(kDrawCommandStart) != std::string::npos) {
      HandleDrawCommand(std::move(line), edges, labels_to_vertices);
    } else if (line.find(kTikzpictureEnd) != std::string::npos) {
      break;
    }
  }

  {
    const auto command = "rm " + filepath + ".tex";
    std::system(command.c_str());
  }

  auto vertices = utils::UnorderedMapToValues(std::move(labels_to_vertices));
  return std::make_unique<Graph>(std::move(vertices), std::move(edges));
}

std::unordered_set<EdgeSptrConst> Graph::CheckKPlanar(
    const std::size_t k) const {
  const auto edges_intersections = CalculateIntersections();

  std::unordered_set<EdgeSptrConst> prohibited_edges;

  for (auto i = 0; i < edges_.size(); i++) {
    if (edges_intersections[i].size() > k) {
      prohibited_edges.insert(edges_[i]);
    }
  }

  return prohibited_edges;
}

std::unordered_set<EdgeSptrConst> Graph::CheckKQuasiPlanar(
    const std::size_t k) const {
  const auto kMinIntersections = k - 1;

  auto edges_intersections = CalculateIntersections();

  EdgeIndices prohibited_edges;

  for (auto i = 0; i < edges_intersections.size(); i++) {
    if (edges_intersections[i].size() >= kMinIntersections) {
      prohibited_edges.insert(i);
    }
  }

  for (auto edge_erased = false; edge_erased; edge_erased = false) {
    if (prohibited_edges.size() < k) {
      return {};
    }

    for (const auto edge : prohibited_edges) {
      for (const auto intersected_edge : edges_intersections[edge]) {
        if (!prohibited_edges.contains(intersected_edge)) {
          edges_intersections[edge].erase(intersected_edge);
        }
      }

      if (edges_intersections[edge].size() < kMinIntersections) {
        edge_erased = true;
        prohibited_edges.erase(edge);
      }
    }
  }

  return GetEdgesByIndices(prohibited_edges);
}

std::unordered_set<EdgeSptrConst> Graph::CheckKSkewness(
    const std::size_t k) const {
  if (edges_.size() <= 1) {
    return {};
  }

  EdgeIndices edges_to_remove;
  auto intersections = CalculateIntersections();
  auto deletions_left = k;
  while (true) {
    auto edge_with_most_intersections = 0;

    for (auto i = 0; i < intersections.size(); i++) {
      if (intersections[i].size() >
          intersections[edge_with_most_intersections].size()) {
        edge_with_most_intersections = i;
      }
    }

    if (intersections[edge_with_most_intersections].size() == 0) {
      break;
    }

    for (const auto intersected_edge :
         intersections[edge_with_most_intersections]) {
      intersections[intersected_edge].erase(edge_with_most_intersections);
    }

    intersections[edge_with_most_intersections].clear();
    edges_to_remove.insert(edge_with_most_intersections);
    deletions_left--;
  }

  // TODO: handle deletions_left

  return GetEdgesByIndices(edges_to_remove);
}

Edges Graph::CheckRAC() const { return CheckACE(std::numbers::pi / 2); }

Edges Graph::CheckACE(const double angle) const {
  return CheckAC(
      [angle](const double other_angle) { return other_angle == angle; });
}

Edges Graph::CheckACLalpha(const double angle) const {
  return CheckAC(
      [angle](const double other_angle) { return other_angle >= angle; });
}

Edges Graph::CheckAC(const ACPredicat &is_satisfying_angle) const {
  const auto intersections =
      CalculateIntersections(IntersectionsPuttingDown::kNonSymmetric);

  std::vector<DirectionVector> direction_vectors;
  direction_vectors.reserve(edges_.size());
  for (const auto &edge : edges_) {
    direction_vectors.push_back(DirectionVector(*edge));
  }

  EdgeIndices prohibited_edges;

  for (auto i = 0; i < intersections.size(); i++) {
    for (const auto j : intersections[i]) {
      const auto angle =
          direction_vectors[i].CalculateAngle(direction_vectors[j]);
      assert(0 <= angle && angle <= std::numbers::pi / 2);
      if (!is_satisfying_angle(angle)) {
        prohibited_edges.insert(i);
        prohibited_edges.insert(j);
      }
    }
  }

  return GetEdgesByIndices(prohibited_edges);
}

std::unordered_set<EdgeSptrConst> Graph::GetEdgesByIndices(
    const EdgeIndices &indices) const {
  std::unordered_set<EdgeSptrConst> edges;
  edges.reserve(indices.size());

  for (const auto index : indices) {
    edges.insert(edges_[index]);
  }

  return edges;
}

std::vector<EdgeIndices> Graph::CalculateIntersections(
    const IntersectionsPuttingDown mode) const {
  std::vector<EdgeIndices> edges_indices(edges_.size());

  for (auto i = 0; i < edges_.size() - 1; i++) {
    for (auto j = i + i; j < edges_.size(); j++) {
      if (edges_[i]->IsIntersect(*edges_[j])) {
        edges_indices[i].insert(j);
        if (mode == IntersectionsPuttingDown::kSymmetric) {
          edges_indices[j].insert(i);
        }
      }
    }
  }

  return edges_indices;
}

}  // namespace graph
