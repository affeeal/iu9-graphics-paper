#pragma once

#include <memory>

namespace graph {

class Vertex;

using VertexSptr = std::shared_ptr<Vertex>;

class Vertex {
public:
  Vertex(double x, double y, const std::string &label);

  bool operator==(const Vertex &other) const;

  const std::string &GetLabel() const { return label_; }
  double GetX() const { return x_; }
  double GetY() const { return y_; }

private:
  std::string label_;
  double x_;
  double y_;
};

} // namespace graph
