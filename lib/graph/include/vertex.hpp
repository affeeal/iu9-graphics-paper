#pragma once

#include <memory>

#include "point.hpp"

namespace graph {

class Vertex;

using VertexSptr = std::shared_ptr<Vertex>;
using VertexSptrConst = std::shared_ptr<const Vertex>;

class Vertex {
 public:
  Vertex() = default;
  Vertex(const Vertex& other);
  Vertex(Vertex&& other);

  Vertex(const double x, const double y);
  Vertex(const double x, const double y, const std::string& label);

  bool operator==(const Vertex& other) const;
  friend std::ostream& operator<<(std::ostream& os, const Vertex& v);

  const std::string& get_label() const { return label_; }
  double get_x() const { return x_; }
  double get_y() const { return y_; }

  bezier::Point AsPoint() const;

 private:
  std::string label_;
  double x_;
  double y_;
};

}  // namespace graph
