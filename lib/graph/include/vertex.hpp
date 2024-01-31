#pragma once

#include <memory>
#include <string_view>

namespace graph {

class IVertex {
public:
  virtual std::string_view GetLabel() const = 0;

  virtual double GetX() const = 0;
  virtual double GetY() const = 0;

  virtual ~IVertex() {}
};

using IVertexUptr = std::unique_ptr<IVertex>;

class Vertex : public IVertex {
public:
  Vertex() = delete;
  explicit Vertex(double x, double y, std::string_view label)
      : x_(std::move(x)), y_(std::move(y)), label_(std::move(label)) {}

  std::string_view GetLabel() const override { return label_; }
  double GetX() const override { return x_; }
  double GetY() const override { return y_; }

private:
  std::string_view label_;
  double x_;
  double y_;
};

} // namespace graph
