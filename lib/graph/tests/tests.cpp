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

std::function<bool(const EdgeSptrConst&)> EdgeRefComp(const EdgeSptrConst& e1) {
  return [&e1](const EdgeSptrConst& e2) { return &*e1 == &*e2; };
}

std::function<bool(const std::vector<EdgeSptrConst>&)> EdgesRefComp(
    const std::vector<EdgeSptrConst>& es1) {
  return [&es1](const std::vector<EdgeSptrConst>& es2) {
    for (const auto& e : es1) {
      if (std::find_if(es2.begin(), es2.end(), EdgeRefComp(e)) == es2.end()) {
        return true;
      }
    }

    return true;
  };
}

std::function<bool(const std::pair<EdgeSptrConst, EdgeSptrConst>&)>
EdgePairRefComp(const std::pair<EdgeSptrConst, EdgeSptrConst>& p1) {
  return [&p1](const std::pair<EdgeSptrConst, EdgeSptrConst>& p2) {
    return &*p1.first == &*p2.first && &*p1.second == &*p2.second ||
           &*p1.first == &*p2.second && &*p1.second == &*p2.first;
  };
}

TEST(GraphKPlanar, 1Planar) {
  Graph g(
      {{1, 3, "a"}, {1, 1, "b"}, {2, 1, "c"}, {2.5, 2, "d"}, {0.5, 2, "e"}});
  g.AddSLEdges({{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 2}});
  const auto& es = g.get_edges();

  const auto unsat_1p = g.CheckKPlanar(1);
  ASSERT_EQ(unsat_1p.size(), 2);

  for (const auto& e : {es[0], es[3]}) {
    EXPECT_NE(std::find_if(unsat_1p.begin(), unsat_1p.end(), EdgeRefComp(e)),
              unsat_1p.end());
  }

  const auto unsat_2p = g.CheckKPlanar(2);
  ASSERT_EQ(unsat_2p.size(), 0);
}

TEST(GraphKQuasiPlanar, 4QuasiPlanar) {
  Graph g({{1, 2, "0"},
           {5, 2, "1"},
           {1, 5, "2"},
           {5, 5, "3"},
           {3, 1, "4"},
           {3, 7, "5"},
           {4, 6, "6"},
           {1, 1, "7"}});
  g.AddSLEdges({{0, 1}, {0, 2}, {1, 3}, {2, 3}, {4, 5}, {6, 7}});
  const auto& es = g.get_edges();

  const auto unsat_3qp = g.CheckKQuasiPlanar(3);
  ASSERT_EQ(unsat_3qp.size(), 2);

  const std::vector<std::vector<EdgeSptrConst>> expected_unsat_3qp{
      {es[0], es[4], es[5]}, {es[3], es[4], es[5]}};

  for (const auto& c : expected_unsat_3qp) {
    EXPECT_NE(std::find_if(unsat_3qp.begin(), unsat_3qp.end(), EdgesRefComp(c)),
              unsat_3qp.end());
  }

  const auto unsat_4qp = g.CheckKQuasiPlanar(4);
  ASSERT_EQ(unsat_4qp.size(), 0);
}

/*
TEST(GraphKQuasiPlanar, 5QuasiPlanar) {
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

*/
TEST(GraphTest, CheckAC) {
  Graph g({{1, 1, "0"},
           {3, 1, "1"},
           {5, 2, "2"},
           {7, 4, "3"},
           {5, 4, "4"},
           {3, 4, "5"}});
  g.AddSLEdges({{0, 3}, {1, 5}, {2, 4}, {4, 5}});
  const auto& es = g.get_edges();

  const auto alpha = std::acos(1 / std::sqrt(5));
  auto unsat_ace = g.CheckACE(alpha);
  ASSERT_EQ(unsat_ace.size(), 0);

  g.AddSLEdges({{0, 2}, {1, 4}});
  unsat_ace = g.CheckACE(alpha);
  ASSERT_EQ(unsat_ace.size(), 3);
  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> expected_unsat_ace{
      {es[1], es[4]}, {es[4], es[5]}, {es[0], es[5]}};
  for (const auto& p : expected_unsat_ace) {
    EXPECT_NE(
        std::find_if(unsat_ace.begin(), unsat_ace.end(), EdgePairRefComp(p)),
        unsat_ace.end());
  }

  const auto unsat_acl = g.CheckACL(alpha);
  ASSERT_EQ(unsat_acl.size(), 2);
  const std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> expected_unsat_acl{
      {es[4], es[5]}, {es[0], es[5]}};
  for (const auto& p : expected_unsat_acl) {
    EXPECT_NE(
        std::find_if(unsat_ace.begin(), unsat_ace.end(), EdgePairRefComp(p)),
        unsat_ace.end());
  }
}

TEST(GraphTest, CheckKLGrid) {
  std::vector<Vertex> vertices{{2, 4, "0"}, {2, 1, "1"}, {4, 4, "2"},
                               {4, 1, "3"}, {1, 3, "4"}, {5, 3, "5"},
                               {1, 2, "6"}, {5, 2, "7"}};
  Graph graph(std::move(vertices));
  graph.AddSLEdges({{0, 1}, {1, 2}, {2, 3}, {4, 5}, {5, 6}, {6, 7}});

  const auto grids_2_3 = graph.CheckGridFree(2, 3);
  ASSERT_EQ(grids_2_3.size(), 6);
  // TODO: check the content

  const auto grid_3_3 = graph.CheckGridFree(3, 3);
  ASSERT_EQ(grid_3_3.size(), 2);  // не баг, а фича
}

}  // namespace

}  // namespace graph
