#include "graph.hpp"

#include <cassert>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "curve.hpp"
#include "edge.hpp"
#include "point.hpp"
#include "utils.hpp"
#include "vertex.hpp"

namespace graph {

namespace {

using LabelToVertex = std::unordered_map<std::string, Vertex>;

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

// TODO: optimize
std::vector<std::string> GetNodes(std::string &&command) {
  std::vector<std::string> nodes;
  std::stringstream ss(std::move(command));

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

std::pair<double, double> NodeToPoint(std::string &&node) {
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

void HandleNodeCommand(std::string &&command, LabelToVertex &vertices) {
  auto nodes = GetNodes(std::move(command));
  assert(nodes.size() == 2); // label and coordinates

  const auto coordinates = NodeToPoint(std::move(nodes.back()));

  vertices[nodes.front()] =
      Vertex(coordinates.first, coordinates.second, nodes.front());
}

std::vector<bezier::Point> GetPoints(std::vector<std::string> &&nodes,
                                     const LabelToVertex &vertices) {
  std::vector<bezier::Point> points;
  points.reserve(nodes.size());

  for (auto i = 0; i < nodes.size(); i++) {
    if (i == 0 || i == nodes.size() - 1) {
      const auto &vertex = vertices.at(std::move(nodes[i]));
      points.push_back(bezier::Point(vertex.GetX(), vertex.GetY()));
    } else {
      const auto coordinates = NodeToPoint(std::move(nodes[i]));
      points.push_back(bezier::Point(coordinates.first, coordinates.second));
    }
  }

  return points;
}

bezier::Curves GetCurves(std::vector<bezier::Point> &&points) {
  bezier::Curves curves;
  curves.reserve(points.size() + 1 / kCurveSize);

  for (auto i = 0; i < curves.size(); i++) {
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

void HandleDrawCommand(std::string &&command, std::vector<Edge> &edges,
                       const LabelToVertex &vertices) {
  auto nodes = GetNodes(std::move(command));

  assert(nodes.size() == kCurveSize ||
         nodes.size() > kCurveSize &&
             nodes.size() % kCurveSize == kCurveSize - 1);

  const auto &start = vertices.at(nodes.front());
  const auto &end = vertices.at(nodes.back());

  auto points = GetPoints(std::move(nodes), vertices);
  auto edge_curves = GetCurves(std::move(points));

  edges.push_back(Edge(start, end, std::move(edge_curves)));
}

} // namespace

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

  LabelToVertex vertices;
  std::vector<Edge> edges;

  // TODO: split
  while (std::getline(tex_file, line)) {
    if (line.find(kNodeCommandStart) != std::string::npos) {
      HandleNodeCommand(std::move(line), vertices);
    } else if (line.find(kDrawCommandStart) != std::string::npos) {
      HandleDrawCommand(std::move(line), edges, vertices);
    } else if (line.find(kTikzpictureEnd) != std::string::npos) {
      break;
    }
  }

  {
    const auto command = "rm " + std::move(filepath + ".tex");
    std::system(command.c_str());
  }

  return std::make_unique<Graph>(utils::UmapToValues(std::move(vertices)),
                                 std::move(edges));
}

} // namespace graph
