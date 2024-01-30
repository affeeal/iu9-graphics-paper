#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "curve.hpp"
#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

namespace {

TEST(PointTest, Addition) {
  EXPECT_EQ(Point(0.1, 0.7) + Point(0.4, 0.3), Point(0.5, 1.0));
}

TEST(PointTest, MultiplicationByANumber) {
  EXPECT_THAT(
      Point(0.4, 0.8) * 0.2,
      testing::FieldsAre(testing::DoubleEq(0.08), testing::DoubleEq(0.16)));
}

TEST(RectangleTest, CalculateArea) {
  const Rectangle rectangle(Point(0.1, 0.6), Point(0.5, 0.3));

  EXPECT_DOUBLE_EQ(rectangle.CalculateArea(), 0.12);
}

TEST(RectangleTest, NoOverlap) {
  const Rectangle first(Point(0.1, 0.5), Point(0.4, 0.2));
  const Rectangle second(Point(0.5, 0.8), Point(0.7, 0.6));

  EXPECT_FALSE(first.IsOverlap(second));
  EXPECT_FALSE(second.IsOverlap(first));
}

TEST(RectangleTest, OverlapTangentially) {
  const Rectangle first(Point(0.1, 0.5), Point(0.4, 0.2));
  const Rectangle second(Point(0.4, 0.8), Point(0.7, 0.5));

  EXPECT_TRUE(first.IsOverlap(second));
  EXPECT_TRUE(second.IsOverlap(first));
}

TEST(RectangleTest, OverlapPartial) {
  const Rectangle first(Point(0.1, 0.5), Point(0.4, 0.2));
  const Rectangle second(Point(0.2, 0.6), Point(0.5, 0.3));

  EXPECT_TRUE(first.IsOverlap(second));
  EXPECT_TRUE(second.IsOverlap(first));
}

TEST(RectangleTest, OverlapNested) {
  const Rectangle first(Point(0.1, 0.5), Point(0.4, 0.2));
  const Rectangle second(Point(0.2, 0.4), Point(0.3, 0.3));

  EXPECT_TRUE(first.IsOverlap(second));
  EXPECT_TRUE(second.IsOverlap(first));
}

TEST(CurveTest, BoundingBoxByTwoPoints) {
  const Curve curve(std::vector<Point>{Point(0.6, 0.5), Point(0.4, 0.3)});
  const auto box = curve.CalculateBoundingBox();

  EXPECT_EQ(box->GetTopLeft(), Point(0.4, 0.5));
  EXPECT_EQ(box->GetBottomRight(), Point(0.6, 0.3));
}

TEST(CurveTest, BoundingBoxByThreePoints) {
  const Curve curve(
      std::vector<Point>{Point(0.2, 0.4), Point(0.4, 0.2), Point(0.6, 0.4)});
  const auto box = curve.CalculateBoundingBox();

  EXPECT_EQ(box->GetTopLeft(), Point(0.2, 0.4));
  EXPECT_EQ(box->GetBottomRight(), Point(0.6, 0.2));
}

TEST(CurveTest, BoundingBoxByFourPoints) {
  const Curve curve(std::vector<Point>{Point(0.1, 0.1), Point(0.4, 0.2),
                                       Point(0.6, 0.6), Point(0.4, 0.4)});
  const auto box = curve.CalculateBoundingBox();

  EXPECT_EQ(box->GetTopLeft(), Point(0.1, 0.6));
  EXPECT_EQ(box->GetBottomRight(), Point(0.6, 0.1));
}

TEST(CurveTest, SplitTwoPointsCurve) {
  const Curve curve(std::vector<Point>{Point(1, 1), Point(7, 3)});
  const auto split = curve.Split(0.5);

  ASSERT_TRUE(split.first);
  ASSERT_TRUE(split.second);

  EXPECT_THAT(split.first->GetPoints(),
              testing::ElementsAre(Point(1, 1), Point(4, 2)));
  EXPECT_THAT(split.second->GetPoints(),
              testing::ElementsAre(Point(7, 3), Point(4, 2)));
}

TEST(CurveTest, SplitThreePointsCurve) {
  const Curve curve(std::vector<Point>{Point(1, 1), Point(7, 3), Point(6, 8)});
  const auto split = curve.Split(0.5);

  ASSERT_TRUE(split.first);
  ASSERT_TRUE(split.second);

  EXPECT_THAT(
      split.first->GetPoints(),
      testing::ElementsAre(Point(1, 1), Point(4, 2), Point(5.25, 3.75)));
  EXPECT_THAT(
      split.second->GetPoints(),
      testing::ElementsAre(Point(6, 8), Point(6.5, 5.5), Point(5.25, 3.75)));
}

TEST(CurveTest, SplitFourPointsCurve) {
  const Curve curve(
      std::vector<Point>{Point(1, 1), Point(7, 3), Point(6, 8), Point(2, 3)});
  const auto split = curve.Split(0.5);

  ASSERT_TRUE(split.first);
  ASSERT_TRUE(split.second);

  EXPECT_THAT(split.first->GetPoints(),
              testing::ElementsAre(Point(1, 1), Point(4, 2), Point(5.25, 3.75),
                                   Point(5.25, 4.625)));
  EXPECT_THAT(split.second->GetPoints(),
              testing::ElementsAre(Point(2, 3), Point(4, 5.5), Point(5.25, 5.5),
                                   Point(5.25, 4.625)));
}

} // namespace

} // namespace bezier
