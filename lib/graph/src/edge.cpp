#include "edge.hpp"

#include <algorithm>
#include <iostream>

#include "curve.hpp"
#include "vector.hpp"

namespace graph {

Edge::Edge(VertexSptrConst start, VertexSptrConst end)
    : start_(std::move(start)), end_(std::move(end)) {
  cs_.push_back(std::make_unique<const bezier::Curve>(
      std::vector<bezier::Point>{*start_, *end_}));
}

Edge::Edge(VertexSptrConst start, VertexSptrConst end,
           std::vector<bezier::CurveUptrConst> &&curves)
    : start_(std::move(start)), end_(std::move(end)), cs_(std::move(curves)) {
  CheckCurvesSizeInvariant();
}

const VertexSptrConst &Edge::get_start() const &noexcept { return start_; }

const VertexSptrConst &Edge::get_end() const &noexcept { return end_; }

const std::vector<bezier::CurveUptrConst> &Edge::get_curves() const &noexcept {
  return cs_;
}

bool Edge::IsIntersect(const Edge &e) const {
  const auto starts_match = (&*start_ == &*e.start_);
  const auto start_mathes_end = (&*start_ == &*e.end_);
  const auto end_matches_start = (&*end_ == &*e.start_);
  const auto ends_match = (&*end_ == &*e.end_);

  const auto back_curve = cs_.size() - 1;
  const auto e_back_curve = e.cs_.size() - 1;

  for (std::size_t i = 0; i < cs_.size(); ++i) {
    for (std::size_t j = 0; j < e.cs_.size(); ++j) {
      if (i == 0 && (j == 0 && starts_match ||
                     j == e_back_curve && start_mathes_end) ||
          i == back_curve && (j == 0 && end_matches_start ||
                              j == e_back_curve && ends_match)) {
        const auto ps = cs_[i]->Intersect(*e.cs_[j]);

        const auto is_boundary = [this](const bezier::Point &p) {
          return start_->IsInNeighborhood(p) || end_->IsInNeighborhood(p);
        };

        if (std::find_if_not(ps.begin(), ps.end(), is_boundary) != ps.end()) {
          return true;
        };
      } else if (cs_[i]->IsIntersect(*e.cs_[j])) {
        return true;
      }
    }
  }

  return false;
}

bool Edge::IsStraightLine() const {
  if (cs_.size() > 1) {
    return false;  // no point in doing straight-line edge from several curves
  }

  const auto &points = cs_.front()->get_points();
  const auto start = points.front();
  const utils::Vector main_dir(points[1] - start);

  for (std::size_t i = 2; i < points.size(); i++) {
    const utils::Vector dir(points[i] - start);
    if (!dir.CollinearTo(main_dir)) {
      return false;
    }
  }

  return true;
}

void Edge::CheckCurvesSizeInvariant() const {
  if (cs_.empty()) {
    throw std::logic_error("Edge curves cannot be empty");
  }
}

void Edge::Dump(std::ostream &os) const {
  os << start_->label << "--" << end_->label;
}

std::ostream &operator<<(std::ostream &os, const Edge &e) {
  e.Dump(os);
  return os;
}

}  // namespace graph
