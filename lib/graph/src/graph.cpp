#include "graph.hpp"

#include <cassert>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "utils.hpp"

namespace graph {

namespace {

using LabelToVertex = std::unordered_map<std::string, VertexUptr>;

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

void HandleNodeCommand(std::string &&command,
                       LabelToVertex &labels_to_vertices) {
  auto nodes = GetNodes(std::move(command));
  assert(nodes.size() == 2); // label and appropriate coordinates

  const auto coordinates = NodeToPoint(std::move(nodes.back()));
  labels_to_vertices[nodes.front()] = std::make_unique<Vertex>(
      coordinates.first, coordinates.second, nodes.front());
}

std::vector<bezier::Point>
NodesToPoints(std::vector<std::string> &&nodes,
              const LabelToVertex &labels_to_vertices) {
  std::vector<bezier::Point> points;
  points.reserve(nodes.size());

  for (auto i = 0; i < nodes.size(); i++) {
    if (i == 0 || i == nodes.size() - 1) {
      const auto &vertex = labels_to_vertices.at(std::move(nodes[i]));
      points.push_back(bezier::Point(vertex->GetX(), vertex->GetY()));
    } else {
      const auto coordinates = NodeToPoint(std::move(nodes[i]));
      points.push_back(bezier::Point(coordinates.first, coordinates.second));
    }
  }

  return points;
}

std::vector<bezier::CurveUptr>
CurvesByPoints(std::vector<bezier::Point> &&points) {
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

void HandleDrawCommand(std::string &&command, std::vector<EdgeUptr> &edges,
                       const LabelToVertex &labels_to_vertices) {
  auto nodes = GetNodes(std::move(command));

  assert(nodes.size() == kCurveSize ||
         nodes.size() > kCurveSize &&
             nodes.size() % kCurveSize == kCurveSize - 1);

  const auto &start = *labels_to_vertices.at(nodes.front());
  const auto &end = *labels_to_vertices.at(nodes.back());

  auto points = NodesToPoints(std::move(nodes), labels_to_vertices);
  auto curves = CurvesByPoints(std::move(points));

  edges.push_back(std::make_unique<Edge>(start, end, std::move(curves)));
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

  LabelToVertex labels_to_vertices;
  std::vector<EdgeUptr> edges;

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

} // namespace graph
