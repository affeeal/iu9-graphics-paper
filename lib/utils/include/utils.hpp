#pragma once

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "edge.hpp"
#include "point.hpp"

namespace utils {

class Vector {
 public:
  Vector() = default;
  explicit Vector(const bezier::Point &point);
  explicit Vector(const graph::Edge &edge);

  double AngleWith(const Vector &other) const;
  double ScalarProduct(const Vector &other) const;
  double Norm() const;
  bool CollinearTo(const Vector &other) const;

 private:
  double x_;
  double y_;
};

std::size_t Factorial(const std::size_t n);

std::size_t Combinations(const std::size_t n, const std::size_t k);

template <typename Value>
std::vector<std::vector<Value>> Combinations(const std::vector<Value> &values,
                                             const std::size_t k) {
  const auto n = values.size();

  if (n < k) {
    throw std::logic_error("N must be greater or equal to K");
  }

  std::vector<std::vector<Value>> combinations;
  combinations.reserve(Combinations(n, k));

  std::vector<bool> mask(k, true);
  mask.resize(n, false);

  do {
    std::vector<Value> combination;
    combination.reserve(k);

    for (auto i = 0; i < n; i++) {
      if (mask[i]) {
        combination.push_back(values[i]);
      }
    }

    combinations.push_back(std::move(combination));
  } while (std::prev_permutation(mask.begin(), mask.end()));

  return combinations;
}

template <typename Value>
std::vector<Value> Intersect(const std::set<Value> &s1,
                             const std::set<Value> &s2) {
  std::vector<Value> intersection;

  auto i1 = s1.begin();
  auto i2 = s2.begin();

  while (i1 != s1.end() && i2 != s2.end()) {
    while (i1 != s1.end() && *i1 < *i2) {
      i1++;
    }

    while (i2 != s2.end() && *i2 < *i1) {
      i2++;
    }

    if (i1 == s1.end() || i2 == s2.end()) {
      break;
    }

    if (*i1 == *i2) {
      intersection.push_back(*i1);
      i1++;
      i2++;
    }
  }

  return intersection;
}

template <typename Set>
std::vector<typename Set::key_type> AsVector(const Set &s) {
  std::vector<typename Set::key_type> result;
  result.reserve(s.size());

  for (const auto &value : s) {
    result.push_back(value);
  }

  return result;
}

template <typename Map>
std::vector<typename Map::mapped_type> ToVector(Map &&m) {
  std::vector<typename Map::mapped_type> values;
  values.reserve(m.size());

  for (auto &[_, value] : m) {
    values.push_back(std::move(value));
  }

  return values;
}

}  // namespace utils
