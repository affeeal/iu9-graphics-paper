#pragma once

#include <boost/functional/hash.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace utils {

std::string GetFileDirectory(const std::string &filename);

std::size_t Factorial(const std::size_t n);

template <typename Value>
std::vector<std::vector<Value>> Combinations(const std::vector<Value> &values,
                                             const std::size_t n,
                                             const std::size_t k) {
  if (n != values.size()) {
    throw std::logic_error("N must be equal to the values' size");
  }

  if (n < k) {
    throw std::logic_error("N must be greater or equal to K");
  }

  std::vector<std::vector<Value>> combinations;
  combinations.reserve(Factorial(n) / Factorial(k) / Factorial(n - k));

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
std::size_t HashSet(const std::set<Value> &s) {
  return boost::hash_range(s.begin(), s.end());
}

template <typename Value>
std::vector<Value> IntersectSets(const std::set<Value> &s1,
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

template <typename Key, typename Value>
std::vector<Value> UnorderedMapToValues(std::unordered_map<Key, Value> &&um) {
  std::vector<Value> values;
  values.reserve(um.size());

  for (auto &record : um) {
    values.push_back(std::move(record.second));
  }

  return values;
}

}  // namespace utils
