#pragma once

#include <memory>

#include "utils.hpp"

namespace graph {

class Vertex;

using VertexUptr = std::unique_ptr<Vertex>;

class Vertex {
public:
  Vertex() : x_(0), y_(0), label_(std::string()) {}
  Vertex(double x, double y, const std::string &label);

  const std::string &GetLabel() const { return label_; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }

  bool operator==(const Vertex &other) const;

private:
  std::string label_;
  double x_;
  double y_;
};

} // namespace graph
