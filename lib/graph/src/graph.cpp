#include "graph.hpp"

#include <cassert>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "curve.hpp"
#include "edge.hpp"
#include "point.hpp"
#include "vertex.hpp"

namespace graph {

namespace {

using LabelToVertex = std::unordered_map<std::string, IVertexUptr>;

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

bezier::Point NodeToPoint(std::string &&node) {
  std::stringstream ss(std::move(node));

  std::string x = "";
  while (!ss.eof()) {
    const auto character = ss.get();
    if (character == kMeasurementUnitStart) {
      break;
    }

    x.push_back(ss.get());
  }

  while (!ss.eof() && ss.get() != kCoordinatesDivider) {
  }

  std::string y = "";
  while (!ss.eof()) {
    const auto character = ss.get();
    if (character == kMeasurementUnitStart) {
      break;
    }

    y.push_back(ss.get());
  }

  if (ss.eof()) {
    throw std::runtime_error("Failed to parse node coordinates");
  }

  return bezier::Point(std::stod(std::move(x)), std::stod(std::move(y)));
}

std::vector<std::string> GetCommandNodes(std::string &&command) {
  std::vector<std::string> nodes;
  std::stringstream ss(std::move(command));

  do {
    while (!ss.eof() && ss.get() != kNodeStart) {
    }

    std::string node = "";
    while (!ss.eof()) {
      const auto character = ss.get();
      if (character == kNodeEnd) {
        break;
      }

      node.push_back(ss.get());
    }

    if (ss.eof()) {
      throw std::runtime_error("Failed to parse command node");
    }

    nodes.push_back(std::move(node));
  } while (ss.get() != kDrawCommandEnd);

  return nodes;
}

void HandleNodeCommand(std::string &&command, LabelToVertex &vertices) {
  std::stringstream ss(std::move(command));

  auto nodes = GetCommandNodes(std::move(command));
  assert(nodes.size() == 2);

  auto &label = nodes.front();
  auto point = NodeToPoint(std::move(nodes.back()));

  vertices[label] = std::make_unique<Vertex>(point.x, point.y, label);
}

std::vector<bezier::Point> GetPoints(std::vector<std::string> &&nodes,
                                     const LabelToVertex &vertices) {
  std::vector<bezier::Point> points;
  points.reserve(nodes.size());

  for (auto i = 0; i < nodes.size(); i++) {
    if (i == 0 || i == nodes.size() - 1) {
      const auto &vertex = vertices.at(std::move(nodes[i]));
      points.push_back(bezier::Point(vertex->GetX(), vertex->GetY()));
    } else {
      points.push_back(NodeToPoint(std::move(nodes[i])));
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

    curves.push_back(
        std::make_unique<bezier::Curve>(std::move(curve_points)));
  }

  return curves;
}

void HandleDrawCommand(std::string &&command, std::vector<IEdgeUptr> &edges,
                       const LabelToVertex &vertices) {
  auto nodes = GetCommandNodes(std::move(command));

  assert(nodes.size() == kCurveSize ||
         nodes.size() > kCurveSize &&
             nodes.size() % kCurveSize == kCurveSize - 1);

  const auto &start = vertices.at(nodes.front());
  const auto &end = vertices.at(nodes.back());

  auto points = GetPoints(std::move(nodes), vertices);
  auto edge_curves = GetCurves(std::move(points));

  edges.push_back(std::make_unique<Edge>(*start, *end, std::move(edge_curves)));
}

template <typename Key, typename Value>
std::vector<Value> UmapToValues(std::unordered_map<Key, Value> &&um) {
  std::vector<Value> values;
  values.resize(um.size());

  for (auto &record : um) {
    values.push_back(std::move(record.second));
  }

  return values;
}

} // namespace

GraphUptr Graph::FromDotFile(std::string filename) {
  auto tex_filename = std::move(filename) + ".tex";

  {
    const auto command = "dot2tex -ftikz -tmath -o " + tex_filename;
    std::system(command.c_str());
  }

  std::ifstream tex_file(tex_filename);
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
  std::vector<IEdgeUptr> edges;

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
    const auto command = "rm " + std::move(tex_filename);
    std::system(command.c_str());
  }

  return std::make_unique<Graph>(UmapToValues(std::move(vertices)),
                                 std::move(edges));
}

} // namespace graph
