#include "utils.hpp"

namespace utils {

std::string GetFileDirectory(const std::string &filename) {
  int end = filename.size() - 1;
  while (end >= 0 && filename[end] != '/') {
    end--;
  }

  return (end < 0) ? "./" : filename.substr(0, end + 1);
}

std::size_t Factorial(const std::size_t n) {
  return (n == 1 || n == 0) ? 1 : n * Factorial(n - 1);
}

}  // namespace utils
