#include <gtest/gtest.h>

#include <algorithm>
#include <cassert>
#include <cmath>

#include "edge.hpp"
#include "graph.hpp"
#include "vertex.hpp"

namespace graph {

namespace {

const std::string kDataPathPrefix = "../../../../data/";

std::function<bool(const EdgeSptrConst&)> EdgeCompare(const EdgeSptrConst& e1) {
  return [&e1](const EdgeSptrConst& e2) { return *e1 == *e2; };
}

std::function<bool(const std::vector<EdgeSptrConst>&)> EdgesComparator(
    const std::vector<EdgeSptrConst>& s1) {
  return [&s1](const std::vector<EdgeSptrConst>& s2) {
    assert(s1.size() == s2.size());

    for (std::size_t i = 0; i < s1.size(); i++) {
      if (*s1[i] != *s2[i]) {
        return false;
      }
    }

    return true;
  };
}

std::function<bool(const std::pair<EdgeSptrConst, EdgeSptrConst>&)>
EdgePairComparator(const std::pair<EdgeSptrConst, EdgeSptrConst>& p1) {
  return [&p1](const std::pair<EdgeSptrConst, EdgeSptrConst>& p2) {
    return *p1.first == *p2.first && *p1.second == *p2.second;
  };
}

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
  const auto& graph_edges = graph.get_edges();

  EXPECT_TRUE(graph_edges[2]->IsIntersect(*graph_edges[3]));
  EXPECT_TRUE(graph_edges[3]->IsIntersect(*graph_edges[2]));

  EXPECT_FALSE(graph_edges[0]->IsIntersect(*graph_edges[2]));
  EXPECT_FALSE(graph_edges[2]->IsIntersect(*graph_edges[0]));

  EXPECT_FALSE(graph_edges[0]->IsIntersect(*graph_edges[1]));
  EXPECT_FALSE(graph_edges[1]->IsIntersect(*graph_edges[0]));
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
  const auto& graph_edges = graph.get_edges();

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

  const auto graph = Graph::FromFile(kDataPathPrefix + "tiny.dot");

  EXPECT_TRUE(expected_graph == *graph);
}

TEST(GraphTest, CheckKPlanar_InvalidK) {
  const auto graph = Graph({}, {});
  EXPECT_ANY_THROW(graph.CheckKPlanar(0));
}

TEST(GraphTest, CheckKPlanar_1Planar) {
  std::vector<VertexSptr> vertices;
  vertices.reserve(5);

  vertices.push_back(std::make_shared<Vertex>(1, 3, "a"));
  vertices.push_back(std::make_shared<Vertex>(1, 1, "b"));
  vertices.push_back(std::make_shared<Vertex>(2, 1, "c"));
  vertices.push_back(std::make_shared<Vertex>(2.5, 2, "d"));
  vertices.push_back(std::make_shared<Vertex>(0.5, 2, "e"));

  std::vector<EdgeSptr> edges;
  edges.reserve(5);

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 3), bezier::Point(1, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[1], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 1), bezier::Point(2, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[1], vertices[2], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(2, 1), bezier::Point(1, 3)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[2], vertices[0], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(2.5, 2), bezier::Point(0.5, 2)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[3], vertices[4], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(std::vector<bezier::Point>{
        bezier::Point(0.5, 2), bezier::Point(2, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[4], vertices[2], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto& edges_ref = graph.get_edges();
  const auto unsatisfying_edges_1_planar = graph.CheckKPlanar(1);

  ASSERT_EQ(unsatisfying_edges_1_planar.size(), 2);
  EXPECT_NE(std::find_if(unsatisfying_edges_1_planar.begin(),
                         unsatisfying_edges_1_planar.end(),
                         EdgeCompare(edges_ref[0])),
            unsatisfying_edges_1_planar.end());
  EXPECT_NE(std::find_if(unsatisfying_edges_1_planar.begin(),
                         unsatisfying_edges_1_planar.end(),
                         EdgeCompare(edges_ref[3])),
            unsatisfying_edges_1_planar.end());

  const auto unsatisfying_edges_2_planar = graph.CheckKPlanar(2);
  ASSERT_EQ(unsatisfying_edges_2_planar.size(), 0);
}

TEST(GraphTest, CheckKQuasiPlanar_InvalidK) {
  const Graph graph({}, {});
  EXPECT_ANY_THROW(graph.CheckKQuasiPlanar(0));
  EXPECT_ANY_THROW(graph.CheckKQuasiPlanar(1));
  EXPECT_ANY_THROW(graph.CheckKQuasiPlanar(2));
}

TEST(GraphTest, CheckKQuasiPlanar_4QuasiPlanar) {
  std::vector<VertexSptr> vertices;
  vertices.reserve(8);

  vertices.push_back(std::make_shared<Vertex>(1, 2, "0"));
  vertices.push_back(std::make_shared<Vertex>(5, 2, "1"));
  vertices.push_back(std::make_shared<Vertex>(1, 5, "2"));
  vertices.push_back(std::make_shared<Vertex>(5, 5, "3"));
  vertices.push_back(std::make_shared<Vertex>(3, 1, "4"));
  vertices.push_back(std::make_shared<Vertex>(3, 7, "5"));
  vertices.push_back(std::make_shared<Vertex>(4, 6, "6"));
  vertices.push_back(std::make_shared<Vertex>(1, 1, "7"));

  std::vector<EdgeSptr> edges;
  edges.reserve(6);

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 2), bezier::Point(5, 2)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[1], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 2), bezier::Point(1, 5)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[0], vertices[2], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(5, 2), bezier::Point(5, 5)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[1], vertices[3], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(1, 5), bezier::Point(5, 5)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[2], vertices[3], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(3, 1), bezier::Point(3, 7)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[4], vertices[5], std::move(curves)));
  }

  {
    std::vector<bezier::CurveUptr> curves;
    curves.push_back(std::make_unique<bezier::Curve>(
        std::vector<bezier::Point>{bezier::Point(4, 6), bezier::Point(1, 1)}));
    edges.push_back(
        std::make_shared<Edge>(vertices[6], vertices[7], std::move(curves)));
  }

  const Graph graph(std::move(vertices), std::move(edges));
  const auto& edges_ref = graph.get_edges();

  const auto _3_cliques = graph.CheckKQuasiPlanar(3);
  ASSERT_EQ(_3_cliques.size(), 2);

  std::vector<std::array<std::size_t, 3>> expected_3_cliques{
      {0, 4, 5},
      {3, 4, 5},
  };

  for (const auto& expected_3_clique : expected_3_cliques) {
    EXPECT_NE(std::find_if(_3_cliques.begin(), _3_cliques.end(),
                           EdgesComparator({edges_ref[expected_3_clique[0]],
                                            edges_ref[expected_3_clique[1]],
                                            edges_ref[expected_3_clique[2]]})),
              _3_cliques.end());
  }

  const auto _4_cliques = graph.CheckKQuasiPlanar(4);
  ASSERT_EQ(_4_cliques.size(), 0);
}

TEST(GraphTest, CheckKQuasiPlanar_5QuasiPlanar) {
  const auto graph = Graph::FromFile(
      kDataPathPrefix + "test_5_quasi_planar.tex", Graph::Filetype::kTex);
  const auto& edges = graph->get_edges();

  const auto _3_cliques = graph->CheckKQuasiPlanar(3);
  ASSERT_EQ(_3_cliques.size(), 8);

  std::vector<std::array<std::size_t, 3>> expected_3_cliques{
      {0, 3, 4}, {0, 1, 2}, {0, 1, 4}, {0, 2, 4},
      {1, 2, 4}, {1, 2, 5}, {2, 4, 5}, {1, 4, 5},
  };

  for (const auto& expected_3_clique : expected_3_cliques) {
    EXPECT_NE(std::find_if(_3_cliques.begin(), _3_cliques.end(),
                           EdgesComparator({edges[expected_3_clique[0]],
                                            edges[expected_3_clique[1]],
                                            edges[expected_3_clique[2]]})),
              _3_cliques.end());
  }

  const auto _4_cliques = graph->CheckKQuasiPlanar(4);
  ASSERT_EQ(_4_cliques.size(), 2);

  std::vector<std::array<std::size_t, 4>> expected_4_cliques{
      {0, 1, 2, 4},
      {1, 2, 4, 5},
  };

  for (const auto& expected_4_clique : expected_4_cliques) {
    EXPECT_NE(std::find_if(_4_cliques.begin(), _4_cliques.end(),
                           EdgesComparator({edges[expected_4_clique[0]],
                                            edges[expected_4_clique[1]],
                                            edges[expected_4_clique[2]],
                                            edges[expected_4_clique[3]]})),
              _4_cliques.end());
  }

  const auto _5_cliques = graph->CheckKQuasiPlanar(5);
  ASSERT_EQ(_5_cliques.size(), 0);
}

TEST(GraphTest, CheckKSkewness_InvalidK) {
  const Graph graph({}, {});
  EXPECT_ANY_THROW(graph.CheckKSkewness(0));
}

TEST(GraphTest, CheckKSkewness_1Skewness) {
  const auto graph = Graph::FromFile(kDataPathPrefix + "test_1_skewness.tex",
                                     Graph::Filetype::kTex);
  const auto& edges = graph->get_edges();

  const auto _1_skewness_unsat = graph->CheckKSkewness(1);
  ASSERT_EQ(_1_skewness_unsat.size(), 0);
}

TEST(GraphTest, CheckKSkewness_2Skewness) {
  const auto graph = Graph::FromFile(kDataPathPrefix + "test_2_skewness.tex",
                                     Graph::Filetype::kTex);
  const auto& edges = graph->get_edges();

  const auto _1_skewness_unsat = graph->CheckKSkewness(1);
  ASSERT_EQ(_1_skewness_unsat.size(), 2);

  const std::vector<std::array<std::size_t, 1>> expected_1_skewness_unsat{
      {2},
      {4},
  };

  for (const auto& s : expected_1_skewness_unsat) {
    EXPECT_NE(std::find_if(_1_skewness_unsat.begin(), _1_skewness_unsat.end(),
                           EdgesComparator({edges[s[0]]})),
              _1_skewness_unsat.end());
  }

  const auto _2_skewness_unsat = graph->CheckKSkewness(2);
  ASSERT_EQ(_2_skewness_unsat.size(), 0);
}

TEST(GraphTest, CheckRAC_Success) {
  const auto graph =
      Graph::FromFile(kDataPathPrefix + "test_rac.tex", Graph::Filetype::kTex);
  const auto rac_unsat_pairs = graph->CheckRAC();
  ASSERT_EQ(rac_unsat_pairs.size(), 0);
}

TEST(GraphTest, CheckRAC_Failure) {
  const auto graph = Graph::FromFile(kDataPathPrefix + "test_non_rac.tex",
                                     Graph::Filetype::kTex);
  const auto& edges = graph->get_edges();

  const auto rac_unsatisfying = graph->CheckRAC();
  ASSERT_EQ(rac_unsatisfying.size(), 3);

  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>>
      expected_rac_unsatisfying{
          {edges[1], edges[8]},
          {edges[2], edges[9]},
          {edges[9], edges[10]},
      };

  for (const auto& pair : expected_rac_unsatisfying) {
    EXPECT_NE(std::find_if(rac_unsatisfying.begin(), rac_unsatisfying.end(),
                           EdgePairComparator(pair)),
              rac_unsatisfying.end());
  }
}

TEST(GraphTest, CheckAC) {
  std::vector<Vertex> vertices{{1, 1, "0"}, {3, 1, "1"}, {5, 2, "2"},
                               {7, 4, "3"}, {5, 4, "4"}, {3, 4, "5"}};
  Graph graph(std::move(vertices));
  graph.AddEdges({{0, 3}, {1, 5}, {2, 4}, {4, 5}});

  const auto alpha = std::acos(1 / std::sqrt(5));
  auto unsatisfying_edge_pairs = graph.CheckACE(alpha);
  ASSERT_EQ(unsatisfying_edge_pairs.size(), 0);

  graph.AddEdges({{0, 2}, {1, 4}});
  unsatisfying_edge_pairs = graph.CheckACE(alpha);
  ASSERT_EQ(unsatisfying_edge_pairs.size(), 3);

  const auto& edges = graph.get_edges();
  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> edge_pairs{
      {edges[1], edges[4]}, {edges[4], edges[5]}, {edges[0], edges[5]}};
  for (const auto& edge_pair : edge_pairs) {
    EXPECT_NE(std::find_if(unsatisfying_edge_pairs.begin(),
                           unsatisfying_edge_pairs.end(),
                           EdgePairComparator(edge_pair)),
              unsatisfying_edge_pairs.end());
  }

  unsatisfying_edge_pairs = graph.CheckACL(alpha);
  ASSERT_EQ(unsatisfying_edge_pairs.size(), 2);
  edge_pairs = {{edges[4], edges[5]}, {edges[0], edges[5]}};
  for (const auto& edge_pair : edge_pairs) {
    EXPECT_NE(std::find_if(unsatisfying_edge_pairs.begin(),
                           unsatisfying_edge_pairs.end(),
                           EdgePairComparator(edge_pair)),
              unsatisfying_edge_pairs.end());
  }
}

}  // namespace

}  // namespace graph
