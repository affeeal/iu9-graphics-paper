#pragma once

#include <algorithm>
#include <set>
#include <stdexcept>
#include <vector>

namespace utils {

static constexpr auto kCombsBadParamsMsg =
    "Combinations n must be greater or equal to k";

constexpr std::size_t Factorial(const std::size_t n) noexcept;

std::size_t Combinations(const std::size_t n, const std::size_t k);

template <typename Value>
std::vector<std::vector<Value>> Combinations(const std::vector<Value> &values,
                                             const std::size_t k) {
  const auto n = values.size();

  if (n < k) {
    throw std::logic_error(kCombsBadParamsMsg);
  }

  std::vector<std::vector<Value>> cs;
  cs.reserve(Combinations(n, k));

  std::vector<bool> mask(k, true);
  mask.resize(n, false);

  do {
    std::vector<Value> c;
    c.reserve(k);

    for (std::size_t i = 0; i < n; ++i) {
      if (mask[i]) {
        c.push_back(values[i]);
      }
    }

    cs.push_back(std::move(c));
  } while (std::prev_permutation(mask.begin(), mask.end()));

  return cs;
}

template <typename Value>
std::vector<Value> Intersect(const std::set<Value> &s1,
                             const std::set<Value> &s2) {
  std::vector<Value> intersection;

  auto it1 = s1.begin();
  auto it2 = s2.begin();

  while (it1 != s1.end() && it2 != s2.end()) {
    while (it1 != s1.end() && *it1 < *it2) {
      ++it1;
    }

    while (it2 != s2.end() && *it2 < *it1) {
      ++it2;
    }

    if (it1 == s1.end() || it2 == s2.end()) {
      break;
    }

    if (*it1 == *it2) {
      intersection.push_back(*it1);
      ++it1;
      ++it2;
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
