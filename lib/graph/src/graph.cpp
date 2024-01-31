#include "graph.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "edge.hpp"
#include "vertex.hpp"

namespace graph {

namespace {

constexpr std::string_view kTikzpictureBeginning = "\\begin{tikzpicture}";
constexpr std::string_view kTikzpictureEnd = "\\end{tikzpicture}";
constexpr std::string_view kNodeCommand = "\\node";
constexpr std::string_view kDrawCommand = "\\draw";

// TODO: rewrite adequately
void HandleNodeCommand(std::unordered_map<std::string, IVertexUptr> &vertices,
                       std::string &&line) {
  // Example line: "\node (main) at (154.6bp,234.0bp) [draw,ellipse] {$main$};"
  std::stringstream ss(std::move(line));

  while (ss.get() != '(') {
  }

  std::string label = "";
  while (ss.peek() != ')') {
    label.push_back(ss.get());
  }

  while (ss.get() != '(') {
  }

  std::string x = "";
  while (ss.peek() != 'b') {
    x.push_back(ss.get());
  }

  while (ss.get() != ',') {
  }

  std::string y = "";
  while (ss.peek() != 'b') {
    y.push_back(ss.get());
  }

  vertices[label] = std::make_unique<Vertex>(std::stod(x), std::stod(y), label);
}

void HandleDrawCommand(
    std::vector<IEdgeUptr> &edges, std::string &&line,
    const std::unordered_map<std::string, IVertexUptr> &vertices) {
  // TODO
}

} // namespace

IGraphUptr Graph::FromDotFile(std::string_view filename) {
  const auto tex_filename = std::string(filename) + ".tex";

  const auto command = "dot2tex -ftikz -tmath -o " + tex_filename;
  std::system(command.c_str());

  std::ifstream tex_file(tex_filename);
  if (!tex_file.is_open()) {
    throw std::runtime_error("Failed to open .tex file");
  }

  std::string line = "";
  while (std::getline(tex_file, line)) {
    if (line.find(kTikzpictureBeginning) != std::string::npos) {
      break;
    }
  }

  if (tex_file.eof()) {
    throw std::runtime_error(
        "Failed to find the beginning of the tikzpicture environment");
  }

  std::unordered_map<std::string, IVertexUptr> vertices;
  std::vector<IEdgeUptr> edges;

  while (std::getline(tex_file, line)) {
    if (line.find(kNodeCommand) != std::string::npos) {
      HandleNodeCommand(vertices, std::move(line));
    } else if (line.find(kDrawCommand) != std::string::npos) {
      HandleDrawCommand(edges, std::move(line), vertices);
    } else if (line.find(kTikzpictureEnd) != std::string::npos) {
      break;
    }
  }

  // TODO
}

} // namespace graph
