#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <set>

#include "utils.hpp"

namespace utils {

namespace {

TEST(UtilsTest, GetIntersection) {
  EXPECT_THAT(Intersect<std::size_t>({}, {}), testing::ElementsAre());

  EXPECT_THAT(Intersect<std::size_t>({1, 2}, {}), testing::ElementsAre());

  EXPECT_THAT(Intersect<std::size_t>({1, 2}, {3, 4, 5}),
              testing::ElementsAre());

  EXPECT_THAT(Intersect<std::size_t>({1, 3, 5, 6}, {2, 4}),
              testing::ElementsAre());

  EXPECT_THAT(Intersect<std::size_t>({1, 3, 4, 6}, {2, 4, 5}),
              testing::ElementsAre(4));

  EXPECT_THAT(Intersect<std::size_t>({1, 2, 4}, {2, 3, 4, 5}),
              testing::ElementsAre(2, 4));

  EXPECT_THAT(Intersect<std::size_t>({1, 2, 4, 5, 6}, {2, 3, 4, 5}),
              testing::ElementsAre(2, 4, 5));

  EXPECT_THAT(Intersect<std::size_t>({1, 2, 3}, {1, 2, 3}),
              testing::ElementsAre(1, 2, 3));
}

TEST(UtilsTest, Combinations) {
  EXPECT_ANY_THROW(Combinations(std::vector<std::size_t>{0}, 2));

  {
    const auto result = Combinations(std::vector<std::size_t>{0, 1, 2}, 3);

    ASSERT_EQ(result.size(), 1);

    EXPECT_THAT(result[0], testing::ElementsAre(0, 1, 2));
  }

  {
    const auto result = Combinations(std::vector<std::size_t>{0, 1, 2, 3}, 3);

    ASSERT_EQ(result.size(), 4);

    EXPECT_THAT(result[0], testing::ElementsAre(0, 1, 2));
    EXPECT_THAT(result[1], testing::ElementsAre(0, 1, 3));
    EXPECT_THAT(result[2], testing::ElementsAre(0, 2, 3));
    EXPECT_THAT(result[3], testing::ElementsAre(1, 2, 3));
  }
}

}  // namespace

}  // namespace utils
