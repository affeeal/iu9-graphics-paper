#include "utils.hpp"

namespace utils {

std::string GetFileDirectory(const std::string &filename) {
  int end = filename.size() - 1;
  while (end >= 0 && filename[end] != '/') {
    end--;
  }

  return (end < 0) ? "./" : filename.substr(0, end + 1);
}

} // namespace utils
