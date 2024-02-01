#pragma once

#include <memory>

#include "utils.hpp"

namespace graph {

class IVertex {
public:
  virtual double GetX() const = 0;
  virtual double GetY() const = 0;
  virtual const std::string &GetLabel() const = 0;

  virtual ~IVertex() {}
};

using IVertexUptr = std::unique_ptr<IVertex>;

class Vertex : public IVertex {
public:
  Vertex() = delete;
  Vertex(double x, double y, const std::string &label);

  const std::string &GetLabel() const override { return label_; }
  double GetX() const override { return x_; }
  double GetY() const override { return y_; }

private:
  std::string label_;
  double x_;
  double y_;
};

} // namespace graph
