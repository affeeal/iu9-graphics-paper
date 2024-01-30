#pragma once

#include <memory>

#include "point.hpp"

namespace bezier {

class IRectangle {
public:
  virtual double CalculateArea() const = 0;
  virtual bool IsOverlap(const IRectangle &other) const = 0;

  virtual const Point &GetTopLeft() const = 0;
  virtual const Point &GetBottomRight() const = 0;

  virtual ~IRectangle(){};
};

using IRectangleUptr = std::unique_ptr<IRectangle>;

class Rectangle : public IRectangle {
public:
  Rectangle() = delete;
  explicit Rectangle(Point &&top_left, Point &&bottom_right);

  double CalculateArea() const override;
  bool IsOverlap(const IRectangle &other) const override;

  const Point &GetTopLeft() const override { return top_left_; }
  const Point &GetBottomRight() const override { return bottom_right_; }

private:
  Point top_left_;
  Point bottom_right_;
};

} // namespace bezier
