#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "curve.hpp"
#include "edge.hpp"
#include "graph.hpp"
#include "vertex.hpp"

const double bezier::kThreshold = 10e-5;

namespace graph {

namespace {

TEST(GraphTest, SimpleGraph) {
  std::vector<IVertexUptr> vertices;
  vertices.reserve(5);
  vertices.push_back(std::make_unique<Vertex>(1, 4, "a"));
  vertices.push_back(std::make_unique<Vertex>(3, 4, "b"));
  vertices.push_back(std::make_unique<Vertex>(1, 1, "c"));
  vertices.push_back(std::make_unique<Vertex>(3, 1, "d"));

  std::vector<IEdgeUptr> edges;
  edges.reserve(4);
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 4), bezier::Point(3, 4)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[0], *vertices[1], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(3, 1)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[2], *vertices[3], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 4), bezier::Point(3, 1)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[0], *vertices[3], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(3, 4)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[2], *vertices[1], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto &graph_edges = graph.GetEdges();

  EXPECT_TRUE(graph_edges[2]->IsIntersect(*graph_edges[3]));
  EXPECT_TRUE(graph_edges[0]->IsIntersect(*graph_edges[2]));
  EXPECT_FALSE(graph_edges[0]->IsIntersect(*graph_edges[1]));
}

TEST(GraphTest, CurveEdges) {
  std::vector<IVertexUptr> vertices;
  vertices.reserve(6);
  vertices.push_back(std::make_unique<Vertex>(1, 4, "first"));
  vertices.push_back(std::make_unique<Vertex>(7, 4, "second"));
  vertices.push_back(std::make_unique<Vertex>(1, 2, "third"));
  vertices.push_back(std::make_unique<Vertex>(5, 3, "fourth"));
  vertices.push_back(std::make_unique<Vertex>(2, 1, "fifth"));
  vertices.push_back(std::make_unique<Vertex>(8, 3, "sixth"));

  std::vector<IEdgeUptr> edges;
  edges.reserve(3);
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(1, 4), bezier::Point(1.5, 6), bezier::Point(2.5, 6),
        bezier::Point(3, 4)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(3, 4), bezier::Point(5, 1), bezier::Point(7, 4)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[0], *vertices[1], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 2), bezier::Point(5, 5)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(5, 5), bezier::Point(6, 4), bezier::Point(5, 3)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[2], *vertices[3], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(2, 1), bezier::Point(3, 2),
                                   bezier::Point(7, 2), bezier::Point(8, 3)}));
    edges.push_back(
        std::make_unique<Edge>(*vertices[4], *vertices[5], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto &graph_edges = graph.GetEdges();

  EXPECT_TRUE(graph_edges[0]->IsIntersect(*graph_edges[1]));
  EXPECT_FALSE(graph_edges[0]->IsIntersect(*graph_edges[2]));
  EXPECT_FALSE(graph_edges[1]->IsIntersect(*graph_edges[2]));
}

} // namespace

} // namespace graph
