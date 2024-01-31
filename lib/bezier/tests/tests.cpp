#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "curve.hpp"
#include "point.hpp"
#include "rectangle.hpp"

namespace bezier {

const double kThreshold = 10e-5;

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

TEST(CurveTest, 2x2NoIntersection) {
  const Curve first(std::vector<Point>{Point(1, 1), Point(2, 6)});
  const Curve second(std::vector<Point>{Point(2, 5), Point(3, 1)});

  EXPECT_FALSE(first.IsIntersect(second, 0.1));
  EXPECT_FALSE(second.IsIntersect(first, 0.1));

  EXPECT_TRUE(first.IsIntersect(second, 1));
  EXPECT_TRUE(second.IsIntersect(first, 1));
}

TEST(CurveTest, 2x2Tangency) {
  const Curve first(std::vector<Point>{Point(1, 1), Point(2, 6)});
  const Curve second(std::vector<Point>{Point(2, 6), Point(3, 1)});

  EXPECT_TRUE(first.IsIntersect(second));
  EXPECT_TRUE(second.IsIntersect(first));
}

TEST(CurveTest, 2x2Intersection) {
  const Curve first(std::vector<Point>{Point(1, 1), Point(2, 6)});
  const Curve second(std::vector<Point>{Point(1, 6), Point(3, 1)});

  EXPECT_TRUE(first.IsIntersect(second));
  EXPECT_TRUE(second.IsIntersect(first));
}

TEST(CurveTest, 3x2NoIntersection) {
  const Curve first(std::vector<Point>{Point(4, 2), Point(6, 8), Point(10, 4)});
  const Curve second(std::vector<Point>{Point(5, 4), Point(6, 5)});

  EXPECT_FALSE(first.IsIntersect(second, 0.01));
  EXPECT_FALSE(second.IsIntersect(first, 0.01));

  EXPECT_TRUE(first.IsIntersect(second, 0.1));
  EXPECT_TRUE(second.IsIntersect(first, 0.1));
}

TEST(CurveTest, 3x2Tangency) {
  const Curve first(std::vector<Point>{Point(4, 2), Point(6, 8), Point(10, 4)});
  const Curve second(std::vector<Point>{Point(5, 5), Point(8, 6)});

  EXPECT_TRUE(first.IsIntersect(second));
  EXPECT_TRUE(second.IsIntersect(first));
}

TEST(CurveTest, 3x2Intersection) {
  const Curve first(std::vector<Point>{Point(4, 2), Point(6, 8), Point(10, 4)});
  const Curve second(std::vector<Point>{Point(4, 3), Point(10, 4)});

  EXPECT_TRUE(first.IsIntersect(second));
  EXPECT_TRUE(second.IsIntersect(first));
}

TEST(CurveTest, 3x2SeveralIntersections) {
  const Curve first(std::vector<Point>{Point(4, 2), Point(6, 8), Point(10, 4)});
  const Curve second(std::vector<Point>{Point(3, 4), Point(5, 2)});

  EXPECT_TRUE(first.IsIntersect(second));
  EXPECT_TRUE(second.IsIntersect(first));
}

TEST(CurveTest, 3x3NoIntersection) {
  const Curve first(std::vector<Point>{Point(4, 2), Point(6, 8), Point(10, 4)});
  const Curve second(std::vector<Point>{Point(5, 2), Point(6, 7), Point(9, 4)});

  EXPECT_FALSE(first.IsIntersect(second, 0.1));
  EXPECT_FALSE(second.IsIntersect(first, 0.1));

  EXPECT_TRUE(first.IsIntersect(second, 1));
  EXPECT_TRUE(second.IsIntersect(first, 1));
}

TEST(CurveTest, 3x3SeveralIntersections) {
  const Curve first(std::vector<Point>{Point(4, 2), Point(6, 8), Point(10, 4)});
  const Curve second(
      std::vector<Point>{Point(5, 2), Point(6, 10), Point(9, 4)});

  EXPECT_TRUE(first.IsIntersect(second));
  EXPECT_TRUE(second.IsIntersect(first));
}

} // namespace

} // namespace bezier
