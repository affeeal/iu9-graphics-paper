#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace utils {

std::string GetFileDirectory(const std::string &filename);

template <typename Key, typename Value>
std::vector<Value> UmapToValues(std::unordered_map<Key, Value> &&um) {
  std::vector<Value> values;
  values.reserve(um.size());

  for (auto &record : um) {
    values.push_back(std::move(record.second));
  }

  return values;
}

} // namespace utils
