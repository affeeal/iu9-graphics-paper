#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "edge.hpp"
#include "graph.hpp"
#include "utils.hpp"

namespace graph {

namespace {

const std::string kDataPathPrefix = "../../../../data/";

std::function<bool(const EdgeSptrConst&)> RefComp(const EdgeSptrConst& e1) {
  return [&e1](const EdgeSptrConst& e2) { return &*e1 == &*e2; };
}

std::function<bool(const std::vector<EdgeSptrConst>&)> RefComp(
    const std::vector<EdgeSptrConst>& es1) {
  return [&es1](const std::vector<EdgeSptrConst>& es2) {
    for (const auto& e : es1) {
      if (std::find_if(es2.begin(), es2.end(), RefComp(e)) == es2.end()) {
        return false;
      }
    }

    return true;
  };
}

std::function<bool(const std::pair<EdgeSptrConst, EdgeSptrConst>&)> RefComp(
    const std::pair<EdgeSptrConst, EdgeSptrConst>& p1) {
  return [&p1](const std::pair<EdgeSptrConst, EdgeSptrConst>& p2) {
    return &*p1.first == &*p2.first && &*p1.second == &*p2.second ||
           &*p1.first == &*p2.second && &*p1.second == &*p2.first;
  };
}

void PrintEdges(const std::vector<EdgeSptrConst>& es) {
  std::cout << '[';

  for (std::size_t i = 0, end = es.size() - 1; i < end; ++i) {
    std::cout << *es[i] << ", ";
  }

  std::cout << *es.back() << "]\n";
}

void PrintEdges(const std::vector<std::vector<EdgeSptrConst>>& ess) {
  std::cout << "]\n";

  for (const auto& es : ess) {
    std::cout << '\t';
    PrintEdges(es);
  }

  std::cout << "[\n";
}

TEST(Graph, 1Planar) {
  Graph g(
      {{1, 3, "a"}, {1, 1, "b"}, {2, 1, "c"}, {2.5, 2, "d"}, {0.5, 2, "e"}});
  g.AddSLEdges({{0, 1}, {1, 2}, {2, 0}, {3, 4}, {4, 2}});
  const auto& es = g.get_edges();

  const auto unsat_1p = g.CheckPlanar(1);
  ASSERT_EQ(unsat_1p.size(), 2);

  for (const auto& e : {es[0], es[3]}) {
    EXPECT_NE(std::find_if(unsat_1p.begin(), unsat_1p.end(), RefComp(e)),
              unsat_1p.end());
  }

  const auto unsat_2p = g.CheckPlanar(2);
  ASSERT_EQ(unsat_2p.size(), 0);
}

TEST(Graph, 4QuasiPlanar) {
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

  const auto unsat_3qp = g.CheckQuasiPlanar(3);
  ASSERT_EQ(unsat_3qp.size(), 2);

  const std::vector<std::vector<EdgeSptrConst>> expected_unsat_3qp{
      {es[0], es[4], es[5]}, {es[3], es[4], es[5]}};

  for (const auto& es : expected_unsat_3qp) {
    EXPECT_NE(std::find_if(unsat_3qp.begin(), unsat_3qp.end(), RefComp(es)),
              unsat_3qp.end());
  }

  const auto unsat_4qp = g.CheckQuasiPlanar(4);
  ASSERT_EQ(unsat_4qp.size(), 0);
}

TEST(Grap, 5QuasiPlanar) {
  Graph g({{2, 7, "0"},
           {1, 5, "1"},
           {1, 3, "2"},
           {3, 2, "3"},
           {4, 1, "4"},
           {5, 1, "5"},
           {6, 2, "6"},
           {6, 3, "7"},
           {9, 3, "8"},
           {9, 5, "9"},
           {8, 9, "10"},
           {6, 6, "11"},
           {5, 8, "12"},
           {9, 8, "13"},
           {4, 6, "14"},
           {5, 7, "15"},
           {2, 6, "16"}});
  g.AddSLEdges({{0, 5},
                {1, 8},
                {2, 9},
                {3, 6},
                {4, 10},
                {7, 11},
                {13, 12},
                {16, 12},
                {14, 15}});
  const auto& es = g.get_edges();

  const auto unsat_3qp = g.CheckQuasiPlanar(3);
  ASSERT_EQ(unsat_3qp.size(), 8);

  const std::vector<std::vector<EdgeSptrConst>> expected_unsat_3qp{
      {es[0], es[3], es[4]}, {es[0], es[1], es[2]}, {es[0], es[1], es[4]},
      {es[0], es[2], es[4]}, {es[1], es[2], es[4]}, {es[1], es[2], es[5]},
      {es[2], es[4], es[5]}, {es[1], es[4], es[5]},
  };

  for (const auto& es : expected_unsat_3qp) {
    EXPECT_NE(std::find_if(unsat_3qp.begin(), unsat_3qp.end(), RefComp(es)),
              unsat_3qp.end());
  }

  const auto unsat_4qp = g.CheckQuasiPlanar(4);
  ASSERT_EQ(unsat_4qp.size(), 2);

  const std::vector<std::vector<EdgeSptrConst>> expected_unsat_4qp{
      {es[0], es[1], es[2], es[4]},
      {es[1], es[2], es[4], es[5]},
  };

  for (const auto& es : expected_unsat_4qp) {
    EXPECT_NE(std::find_if(unsat_4qp.begin(), unsat_4qp.end(), RefComp(es)),
              unsat_4qp.end());
  }

  const auto unsat_5qp = g.CheckQuasiPlanar(5);
  ASSERT_EQ(unsat_5qp.size(), 0);
}

TEST(Graph, CheckSkewness) {
  Graph g({{1, 1, "0"},
           {3, 2, "1"},
           {4, 2, "2"},
           {7, 6, "3"},
           {5, 5, "4"},
           {5, 6, "5"},
           {4, 6, "6"},
           {3, 5, "7"}});
  g.AddSLEdges({{0, 4}, {1, 3}, {2, 6}, {5, 7}});

  auto unsat_1s = g.CheckSkewness(1);
  ASSERT_EQ(unsat_1s.size(), 0);

  g.AddVertices({{3, 6, "8"}, {5, 3, "9"}});
  g.AddSLEdge(8, 9);
  const auto& es = g.get_edges();

  unsat_1s = g.CheckSkewness(1);
  PrintEdges(unsat_1s);
  ASSERT_EQ(unsat_1s.size(), 1);

  const std::vector<std::vector<EdgeSptrConst>> expected_unsat_1s{
      {es[2], es[4]}};

  for (const auto& es : expected_unsat_1s) {
    EXPECT_NE(std::find_if(unsat_1s.begin(), unsat_1s.end(), RefComp(es)),
              unsat_1s.end());
  }

  const auto unsat_2s = g.CheckSkewness(2);
  ASSERT_EQ(unsat_2s.size(), 0);
}

TEST(Graph, CheckRAC) {
  Graph g({{2, 3, "0"},
           {4, 1, "1"},
           {9, 2, "2"},
           {12, 5, "3"},
           {10, 5, "4"},
           {7, 6, "5"},
           {6, 3, "6"},
           {4, 5, "7"}});
  g.AddSLEdges(
      {{0, 1}, {1, 3}, {2, 5}, {4, 6}, {7, 6}, {7, 0}, {6, 0}, {1, 7}});

  auto unsat_rac = g.CheckRAC();
  ASSERT_EQ(unsat_rac.size(), 0);

  g.AddSLEdges({{2, 4}, {7, 4}, {6, 5}});
  const auto& es = g.get_edges();

  unsat_rac = g.CheckRAC();
  ASSERT_EQ(unsat_rac.size(), 3);

  const std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> expected_unsat_rac{
      {es[1], es[8]}, {es[2], es[9]}, {es[9], es[10]}};

  for (const auto& p : expected_unsat_rac) {
    EXPECT_NE(std::find_if(unsat_rac.begin(), unsat_rac.end(), RefComp(p)),
              unsat_rac.end());
  }
}

TEST(Graph, CheckAC) {
  Graph g({{1, 1, "0"},
           {3, 1, "1"},
           {5, 2, "2"},
           {7, 4, "3"},
           {5, 4, "4"},
           {3, 4, "5"}});
  g.AddSLEdges({{0, 3}, {1, 5}, {2, 4}, {4, 5}});

  const auto alpha = std::acos(1 / std::sqrt(5));
  auto unsat_ace = g.CheckACE(alpha);
  ASSERT_EQ(unsat_ace.size(), 0);

  g.AddSLEdges({{0, 2}, {1, 4}});
  const auto& es = g.get_edges();

  unsat_ace = g.CheckACE(alpha);
  ASSERT_EQ(unsat_ace.size(), 3);

  const std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> expected_unsat_ace{
      {es[1], es[4]}, {es[4], es[5]}, {es[0], es[5]}};

  for (const auto& p : expected_unsat_ace) {
    EXPECT_NE(std::find_if(unsat_ace.begin(), unsat_ace.end(), RefComp(p)),
              unsat_ace.end());
  }

  const auto unsat_acl = g.CheckACL(alpha);
  ASSERT_EQ(unsat_acl.size(), 2);

  const std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> expected_unsat_acl{
      {es[4], es[5]}, {es[0], es[5]}};

  for (const auto& p : expected_unsat_acl) {
    EXPECT_NE(std::find_if(unsat_ace.begin(), unsat_ace.end(), RefComp(p)),
              unsat_ace.end());
  }
}

TEST(Graph, CheckKLGrid) {
  Graph g({{2, 4, "0"},
           {2, 1, "1"},
           {4, 4, "2"},
           {4, 1, "3"},
           {1, 3, "4"},
           {5, 3, "5"},
           {1, 2, "6"},
           {5, 2, "7"}});
  g.AddSLEdges({{0, 1}, {1, 2}, {2, 3}, {4, 5}, {5, 6}, {6, 7}});

  const auto grids_2_3 = g.CheckGridFree(2, 3);
  ASSERT_EQ(grids_2_3.size(), 6);

  const auto grid_3_3 = g.CheckGridFree(3, 3);
  ASSERT_EQ(grid_3_3.size(), 2);
}

TEST(Graph, K10) {
  const auto g = Graph::Graphviz(kDataPathPrefix + "k10.dot");
  const auto& vs = g->get_vertices();
  const auto& es = g->get_edges();

  ASSERT_EQ(vs.size(), 10);
  ASSERT_EQ(es.size(), 45);  // 1 + 2 + ... + 9

  const auto unsat_6p = g->CheckPlanar(6);
  // TODO: check

  const auto unsat_3qp = g->CheckQuasiPlanar(3);
  // TODO: check

  const auto unsat_20s = g->CheckSkewness(20);
  // TODO: check

  const auto unsat_grid = g->CheckGridFree(3, 3);
  // TODO: check
}

}  // namespace

}  // namespace graph
