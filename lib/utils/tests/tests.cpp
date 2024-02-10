#include <gtest/gtest.h>

#include "utils.hpp"

namespace utils {

namespace {

TEST(UtilsTest, GetFileDirectory) {
  EXPECT_EQ(GetFileDirectory("/dir1/dir2/file.dot"), "/dir1/dir2/");
  EXPECT_EQ(GetFileDirectory("/dir1/file.dot"), "/dir1/");
  EXPECT_EQ(GetFileDirectory("/file.dot"), "/");
  EXPECT_EQ(GetFileDirectory("file.dot"), "./");
}

}  // namespace

}  // namespace utils
