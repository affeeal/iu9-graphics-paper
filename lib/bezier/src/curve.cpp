#include "curve.hpp"

#include <algorithm>
#include <limits>

namespace bezier {

static constexpr auto kCurveCenterT = 0.5;

Curve::Curve(const std::vector<Point> &ps) : ps_(ps) {
  CheckPointsSizeInvariant();
}

Curve::Curve(std::vector<Point> &&ps) : ps_(std::move(ps)) {
  CheckPointsSizeInvariant();
}

bool Curve::operator==(const Curve &rhs) const { return ps_ == rhs.ps_; }

const std::vector<Point> &Curve::get_points() const &noexcept { return ps_; }

std::ostream &operator<<(std::ostream &os, const Curve &rhs) {
  rhs.Dump(os);
  return os;
}

Rectangle Curve::BoundingRectangle() const {
  auto leftmost_x = std::numeric_limits<double>::max();
  auto topmost_y = std::numeric_limits<double>::min();
  auto rightmost_x = std::numeric_limits<double>::min();
  auto bottommost_y = std::numeric_limits<double>::max();

  for (const auto &p : ps_) {
    if (p.x < leftmost_x) {
      leftmost_x = p.x;
    }

    if (p.y > topmost_y) {
      topmost_y = p.y;
    }

    if (p.x > rightmost_x) {
      rightmost_x = p.x;
    }

    if (p.y < bottommost_y) {
      bottommost_y = p.y;
    }
  }

  return {{leftmost_x, topmost_y}, {rightmost_x, bottommost_y}};
}

std::pair<std::unique_ptr<Curve>, std::unique_ptr<Curve>> Curve::Split(
    const double t) const {
  std::vector<Point> ps1;
  ps1.reserve(ps_.size());

  std::vector<Point> ps2;
  ps2.reserve(ps_.size());

  SplitDeCasteljau(ps1, ps2, ps_, t);

  return std::make_pair(std::make_unique<Curve>(std::move(ps1)),
                        std::make_unique<Curve>(std::move(ps2)));
}

bool Curve::IsIntersect(const Curve &c, const double eps) const {
  return IsIntersect(*this, c, eps);
}

std::vector<Point> Curve::Intersect(const Curve &c, const double eps) const {
  std::vector<Point> intersections;
  Intersect(intersections, *this, c, eps);
  return intersections;
}

void Curve::Intersect(std::vector<Point> &ps, const Curve &c1, const Curve &c2,
                      const double eps) const {
  const auto r1 = c1.BoundingRectangle();
  const auto r2 = c2.BoundingRectangle();

  if (!r1.IsOverlap(r2)) {
    return;
  }

  if (CompletionMetric(r1, r2) < eps) {
    // One of the possible intersection approximation
    const auto intersection = r1.Center().CenterWith(r2.Center());

    const auto is_in_neighbourhood = [&intersection](const Point &p) {
      return intersection.IsInNeighborhood(p);
    };

    if (std::find_if(ps.begin(), ps.end(), is_in_neighbourhood) == ps.end()) {
      ps.push_back(intersection);
    }

    return;
  }

  const auto s1 = c1.Split(kCurveCenterT);
  const auto s2 = c2.Split(kCurveCenterT);

  Intersect(ps, *s1.first, *s2.first, eps);
  Intersect(ps, *s1.first, *s2.second, eps);
  Intersect(ps, *s1.second, *s2.first, eps);
  Intersect(ps, *s1.second, *s2.second, eps);
}

void Curve::Dump(std::ostream &os) const {
  os << '[';

  for (std::size_t i = 0, end = ps_.size() - 1; i < end; ++i) {
    os << ps_[i] << ", ";
  }

  os << ps_.back() << ']';
}

void Curve::CheckPointsSizeInvariant() const {
  if (ps_.size() < 2) {
    throw std::logic_error("Curve points size invariant violated");
  }
}

void Curve::SplitDeCasteljau(std::vector<Point> &ps1, std::vector<Point> &ps2,
                             const std::vector<Point> &ps,
                             const double t) const {
  if (ps.size() == 1) {
    ps1.push_back(ps.front());
    ps2.push_back(ps.front());
    return;
  }

  const auto new_size = ps.size() - 1;
  std::vector<Point> new_ps;
  new_ps.reserve(new_size);

  ps1.push_back(ps.front());
  ps2.push_back(ps.back());

  for (std::size_t i = 0; i < new_size; i++) {
    new_ps.push_back(ps[i] * (1 - t) + ps[i + 1] * t);
  }

  SplitDeCasteljau(ps1, ps2, new_ps, t);
}

bool Curve::IsIntersect(const Curve &c1, const Curve &c2,
                        const double eps) const {
  const auto r1 = c1.BoundingRectangle();
  const auto r2 = c2.BoundingRectangle();

  if (!r1.IsOverlap(r2)) {
    return false;
  }

  if (CompletionMetric(r1, r2) < eps) {
    return true;
  }

  const auto s1 = c1.Split(kCurveCenterT);
  const auto s2 = c2.Split(kCurveCenterT);

  return IsIntersect(*s1.first, *s2.first, eps) ||
         IsIntersect(*s1.first, *s2.second, eps) ||
         IsIntersect(*s1.second, *s2.first, eps) ||
         IsIntersect(*s1.second, *s2.second, eps);
}

double Curve::CompletionMetric(const Rectangle &r1,
                               const Rectangle &r2) const noexcept {
  return r1.Perimeter() + r2.Perimeter();
}

}  // namespace bezier
