#include <gtest/gtest.h>

#include <algorithm>

#include "graph.hpp"
#include "vertex.hpp"

const double bezier::kThreshold = 10e-5;

namespace graph {

namespace {

const std::string kDataPathPrefix = "../../../../data/";

TEST(GraphTest, SimpleGraphBuilding) {
  std::vector<VertexSptr> vertices;
  vertices.reserve(4);

  vertices.push_back(std::make_shared<Vertex>(1, 4, "a"));
  vertices.push_back(std::make_shared<Vertex>(3, 4, "b"));
  vertices.push_back(std::make_shared<Vertex>(1, 1, "c"));
  vertices.push_back(std::make_shared<Vertex>(3, 1, "d"));

  std::vector<EdgeSptr> edges;
  edges.reserve(4);

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 4), bezier::Point(3, 4)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[1], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(3, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[2], vertices[3], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 4), bezier::Point(3, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[3], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(3, 4)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[2], vertices[1], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto &graph_edges = graph.GetEdges();

  EXPECT_TRUE(graph_edges[2]->IsIntersect(*graph_edges[3]));
  EXPECT_TRUE(graph_edges[0]->IsIntersect(*graph_edges[2]));
  EXPECT_FALSE(graph_edges[0]->IsIntersect(*graph_edges[1]));
}

TEST(GraphTest, CurvedGraphBuilding) {
  std::vector<VertexSptr> vertices;
  vertices.reserve(6);

  vertices.push_back(std::make_shared<Vertex>(1, 4, "first"));
  vertices.push_back(std::make_shared<Vertex>(7, 4, "second"));
  vertices.push_back(std::make_shared<Vertex>(1, 2, "third"));
  vertices.push_back(std::make_shared<Vertex>(5, 3, "fourth"));
  vertices.push_back(std::make_shared<Vertex>(2, 1, "fifth"));
  vertices.push_back(std::make_shared<Vertex>(8, 3, "sixth"));

  std::vector<EdgeSptr> edges;
  edges.reserve(3);

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(1, 4), bezier::Point(1.5, 6), bezier::Point(2.5, 6),
        bezier::Point(3, 4)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(3, 4), bezier::Point(5, 1), bezier::Point(7, 4)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[1], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 2), bezier::Point(5, 5)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(5, 5), bezier::Point(6, 4), bezier::Point(5, 3)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[2], vertices[3], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(2, 1), bezier::Point(3, 2),
                                   bezier::Point(7, 2), bezier::Point(8, 3)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[4], vertices[5], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto &graph_edges = graph.GetEdges();

  EXPECT_TRUE(graph_edges[0]->IsIntersect(*graph_edges[1]));
  EXPECT_FALSE(graph_edges[0]->IsIntersect(*graph_edges[2]));
  EXPECT_FALSE(graph_edges[1]->IsIntersect(*graph_edges[2]));
}

TEST(GraphTest, TinyFromFile) {
  std::vector<VertexSptr> expected_vertices;
  expected_vertices.reserve(3);

  expected_vertices.push_back(std::make_shared<Vertex>(27.0, 162.0, "a"));
  expected_vertices.push_back(std::make_shared<Vertex>(27.0, 90.0, "b"));
  expected_vertices.push_back(std::make_shared<Vertex>(54.0, 18.0, "c"));

  std::vector<EdgeSptr> expected_edges;
  expected_edges.reserve(4);

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(27.0, 162.0), bezier::Point(20.297, 136.51),
        bezier::Point(20.048, 126.85), bezier::Point(27.0, 90.0)}));
    expected_edges.push_back(std::make_shared<Edge>(
        expected_vertices[0], expected_vertices[1], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(27.0, 90.0), bezier::Point(33.714, 115.83),
        bezier::Point(33.948, 125.37), bezier::Point(27.0, 162.0)}));
    expected_edges.push_back(std::make_shared<Edge>(
        expected_vertices[1], expected_vertices[0], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(27.0, 90.0), bezier::Point(36.514, 64.335),
        bezier::Point(40.334, 54.431), bezier::Point(54.0, 18.0)}));
    expected_edges.push_back(std::make_shared<Edge>(
        expected_vertices[1], expected_vertices[2], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(54.0, 18.0), bezier::Point(64.687, 53.977),
        bezier::Point(70.486, 83.656), bezier::Point(63.0, 108.0)}));
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(63.0, 108.0), bezier::Point(59.714, 118.69),
        bezier::Point(53.46, 129.15), bezier::Point(27.0, 162.0)}));
    expected_edges.push_back(std::make_shared<Edge>(
        expected_vertices[2], expected_vertices[0], std::move(curves)));
  }

  const Graph expected_graph(std::move(expected_vertices),
                             std::move(expected_edges));

  const auto graph = Graph::FromDotFile(kDataPathPrefix + "tiny.dot");

  EXPECT_TRUE(expected_graph == *graph);
}

TEST(GraphTest, CheckKPlanar_InvalidK) {
  const auto graph = Graph({}, {});
  EXPECT_ANY_THROW(graph.CheckKPlanar(0));
}

TEST(GraphTesT, CheckKPlanar_1Planar) {
  std::vector<VertexSptr> vertices;
  vertices.reserve(10);

  vertices.push_back(std::make_shared<Vertex>(0, 1, "0"));
  vertices.push_back(std::make_shared<Vertex>(2, 3, "1"));
  vertices.push_back(std::make_shared<Vertex>(3, 2, "2"));
  vertices.push_back(std::make_shared<Vertex>(0, 3, "3"));
  vertices.push_back(std::make_shared<Vertex>(3, 0, "4"));
  vertices.push_back(std::make_shared<Vertex>(1, 0, "5"));
  vertices.push_back(std::make_shared<Vertex>(1, 1, "6"));
  vertices.push_back(std::make_shared<Vertex>(3, 1, "7"));
  vertices.push_back(std::make_shared<Vertex>(2, 2, "8"));
  vertices.push_back(std::make_shared<Vertex>(3, 3, "9"));

  std::vector<EdgeSptr> edges;
  edges.reserve(6);

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(0, 1), bezier::Point(2, 3)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[1], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(2, 3), bezier::Point(3, 2)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[1], vertices[2], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(0, 3), bezier::Point(3, 0)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[3], vertices[4], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 0), bezier::Point(1, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[5], vertices[6], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(3, 1), bezier::Point(2, 2)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[7], vertices[8], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(2, 2), bezier::Point(3, 3)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[8], vertices[9], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto unsatisfying_edges = graph.CheckKPlanar(1);

  EXPECT_TRUE(unsatisfying_edges.empty());
}

}  // namespace

}  // namespace graph
