#include "utils.hpp"

namespace utils {

constexpr std::size_t Factorial(const std::size_t n) noexcept {
  return (n == 1 || n == 0) ? 1 : n * Factorial(n - 1);
}

std::size_t Combinations(const std::size_t n, const std::size_t k) {
  if (n < k) {
    throw std::logic_error(kCombsBadParamsMsg);
  }

  return Factorial(n) / Factorial(k) / Factorial(n - k);
}

}  // namespace utils
