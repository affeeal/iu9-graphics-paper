#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <iostream>
#include <ostream>

#include "curve.hpp"
#include "edge.hpp"
#include "graph.hpp"
#include "point.hpp"
#include "vertex.hpp"

const double bezier::kThreshold = 10e-5;

namespace graph {

namespace {

const std::string kDataPathPrefix = "../../../../data/";

TEST(GraphTest, SimpleGraph) {
  std::vector<Vertex> vertices{Vertex(1, 4, "a"), Vertex(3, 4, "b"),
                               Vertex(1, 1, "c"), Vertex(3, 1, "d")};

  std::vector<Edge> edges;
  edges.reserve(4);
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 4), bezier::Point(3, 4)}));
    edges.push_back(Edge(vertices[0], vertices[1], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(3, 1)}));
    edges.push_back(Edge(vertices[2], vertices[3], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 4), bezier::Point(3, 1)}));
    edges.push_back(Edge(vertices[0], vertices[3], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(3, 4)}));
    edges.push_back(Edge(vertices[2], vertices[1], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto &graph_edges = graph.GetEdges();

  EXPECT_TRUE(graph_edges[2].IsIntersect(graph_edges[3]));
  EXPECT_TRUE(graph_edges[0].IsIntersect(graph_edges[2]));
  EXPECT_FALSE(graph_edges[0].IsIntersect(graph_edges[1]));
}

TEST(GraphTest, CurveEdges) {
  std::vector<Vertex> vertices{Vertex(1, 4, "first"), Vertex(7, 4, "second"),
                               Vertex(1, 2, "third"), Vertex(5, 3, "fourth"),
                               Vertex(2, 1, "fifth"), Vertex(8, 3, "sixth")};

  std::vector<Edge> edges;
  edges.reserve(3);
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(1, 4), bezier::Point(1.5, 6), bezier::Point(2.5, 6),
        bezier::Point(3, 4)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(3, 4), bezier::Point(5, 1), bezier::Point(7, 4)}));
    edges.push_back(Edge(vertices[0], vertices[1], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 2), bezier::Point(5, 5)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(5, 5), bezier::Point(6, 4), bezier::Point(5, 3)}));
    edges.push_back(Edge(vertices[2], vertices[3], std::move(curves)));
  }
  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(2, 1), bezier::Point(3, 2),
                                   bezier::Point(7, 2), bezier::Point(8, 3)}));
    edges.push_back(Edge(vertices[4], vertices[5], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto &graph_edges = graph.GetEdges();

  EXPECT_TRUE(graph_edges[0].IsIntersect(graph_edges[1]));
  EXPECT_FALSE(graph_edges[0].IsIntersect(graph_edges[2]));
  EXPECT_FALSE(graph_edges[1].IsIntersect(graph_edges[2]));
}

TEST(GraphTest, TinyFromFile) {
  const auto graph = Graph::FromDotFile(kDataPathPrefix + "tiny.dot");

  const auto &vertices = graph->GetVertices();
  EXPECT_EQ(vertices.size(), 3);

  std::array<Vertex, 3> expected_vertices{
      Vertex(27.0, 162.0, "a"),
      Vertex(27.0, 90.0, "b"),
      Vertex(54.0, 18.0, "c"),
  };

  for (const auto &expected_vertex : expected_vertices) {
    EXPECT_NE(std::find(vertices.begin(), vertices.end(), expected_vertex),
              vertices.end());
  }

  std::vector<Edge> expected_edges;
  expected_edges.reserve(4);

  {
    bezier::Curves curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(27.0, 162.0), bezier::Point(20.297, 136.51),
        bezier::Point(20.048, 126.85), bezier::Point(27.0, 90.0)}));
    expected_edges.push_back(
        Edge(expected_vertices[0], expected_vertices[1], std::move(curves)));
  }

  const auto &edges = graph->GetEdges();
  EXPECT_EQ(edges.size(), 4);

  for (const auto &expected_edge : expected_edges) {
    EXPECT_NE(std::find(edges.begin(), edges.end(), expected_edge),
              edges.end());
  }
}

} // namespace

} // namespace graph
