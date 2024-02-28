#include "graph.hpp"

#include <boost/functional/hash.hpp>
#include <fstream>

#include "edge.hpp"
#include "utils.hpp"
#include "vector.hpp"

namespace graph {

namespace {

constexpr std::string_view kTikzpictureStart = "\\begin{tikzpicture}";
constexpr std::string_view kTikzpictureEnd = "\\end{tikzpicture}";
constexpr std::string_view kNodeCommandStart = "\\node";
constexpr std::string_view kDrawCommandStart = "\\draw";

constexpr auto kNodeStart = '(';
constexpr auto kNodeEnd = ')';
constexpr auto kMeasurementUnitStart = 'b';
constexpr auto kCoordinatesDivider = ',';
constexpr auto kDrawCommandEnd = ';';

constexpr std::size_t kTexCurveSize = 4;

std::vector<std::string> CommandToNodes(std::string &&command) {
  std::vector<std::string> nodes;
  std::stringstream ss(command);

  while (true) {
    while (!ss.eof() && ss.get() != kNodeStart) {
    }

    if (ss.eof()) {
      break;
    }

    std::string node = "";
    while (!ss.eof()) {
      const auto c = ss.get();
      if (c == kNodeEnd) {
        break;
      }

      node.push_back(c);
    }

    if (ss.eof()) {
      throw std::runtime_error("Unexpected end while parsing node command");
    }

    nodes.push_back(std::move(node));
  }

  return nodes;
}

bezier::Point NodeToPoint(std::string &&node) {
  std::stringstream ss(std::move(node));

  std::string x = "";
  while (!ss.eof()) {
    const auto c = ss.get();
    if (c == kMeasurementUnitStart) {
      break;
    }

    x.push_back(c);
  }

  while (!ss.eof() && ss.get() != kCoordinatesDivider) {
  }

  std::string y = "";
  while (!ss.eof()) {
    const auto c = ss.get();
    if (c == kMeasurementUnitStart) {
      break;
    }

    y.push_back(c);
  }

  if (ss.eof()) {
    throw std::runtime_error("Unexpected end while parsing node coordinates");
  }

  return {std::stod(x), std::stod(y)};
}

void HandleNodeCommand(
    std::string &&command,
    std::unordered_map<std::string, VertexSptrConst> &labels_to_vertices) {
  auto nodes = CommandToNodes(std::move(command));
  assert(nodes.size() == 2);  // label and appropriate coordinates

  const auto p = NodeToPoint(std::move(nodes.back()));
  labels_to_vertices[nodes.front()] =
      std::make_shared<const Vertex>(p.x, p.y, nodes.front());
}

std::vector<bezier::Point> NodesToPoints(
    std::vector<std::string> &&nodes,
    const std::unordered_map<std::string, VertexSptrConst>
        &labels_to_vertices) {
  std::vector<bezier::Point> ps;
  ps.reserve(nodes.size());

  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (i == 0 || i == nodes.size() - 1) {
      const auto &v = labels_to_vertices.at(nodes[i]);
      ps.push_back(bezier::Point(v->x, v->y));
    } else {
      ps.push_back(NodeToPoint(std::move(nodes[i])));
    }
  }

  return ps;
}

std::vector<bezier::CurveUptrConst> CurvesByPoints(
    const std::vector<bezier::Point> &points) {
  std::vector<bezier::CurveUptrConst> cs;
  cs.reserve((points.size() + 1) / kTexCurveSize);

  for (std::size_t i = 0; i < cs.capacity(); ++i) {
    std::vector<bezier::Point> ps;
    ps.reserve(kTexCurveSize);

    for (std::size_t j = 0; j < kTexCurveSize; ++j) {
      ps.push_back(points[i * (kTexCurveSize - 1) + j]);
    }

    cs.push_back(std::make_unique<const bezier::Curve>(std::move(ps)));
  }

  return cs;
}

void HandleDrawCommand(std::string &&command, std::vector<EdgeSptrConst> &es,
                       const std::unordered_map<std::string, VertexSptrConst>
                           &labels_to_vertices) {
  auto nodes = CommandToNodes(std::move(command));
  if (nodes.size() == 1) {
    return;  // ignore edge attribute
  }

  assert(nodes.size() == kTexCurveSize ||
         nodes.size() > kTexCurveSize &&
             nodes.size() % kTexCurveSize == kTexCurveSize - 1);

  const auto &start = labels_to_vertices.at(nodes.front());
  const auto &end = labels_to_vertices.at(nodes.back());

  const auto points = NodesToPoints(std::move(nodes), labels_to_vertices);
  auto curves = CurvesByPoints(points);

  es.push_back(std::make_shared<const Edge>(start, end, std::move(curves)));
}

enum class CliqueComplementResult {
  kFailure,
  kSuccess,
};

std::unordered_set<std::size_t> kQuasiPlanarCandidates(
    std::vector<std::unordered_set<std::size_t>> &intersections,
    const std::size_t k) {
  std::unordered_set<std::size_t> candidates;

  for (std::size_t i = 0; i < intersections.size(); ++i) {
    if (intersections[i].size() >= k - 1) {
      candidates.insert(i);
    }
  }

  for (auto candidate_deleted = false; candidate_deleted; candidate_deleted = false) {
    if (candidates.size() < k) {
      return {};
    }

    for (const auto candidate : candidates) {
      for (const auto edge : intersections[candidate]) {
        if (!candidates.contains(edge)) {
          intersections[candidate].erase(edge);
        }
      }

      if (intersections[candidate].size() < k - 1) {
        candidate_deleted = true;
        candidates.erase(candidate);
      }
    }
  }

  return candidates;
}

CliqueComplementResult PickCliques(
    std::unordered_map<std::size_t, std::unique_ptr<std::set<std::size_t>>>
        &cliques,
    std::set<std::size_t> current_clique,  // exactly a copy
    const std::vector<std::unordered_set<std::size_t>> &intersections,
    const std::size_t current_edge, const std::size_t k) {
  for (const auto edge : current_clique) {
    if (!intersections[current_edge].contains(edge)) {
      return CliqueComplementResult::kFailure;
    }
  }

  current_clique.insert(current_edge);
  auto is_final_edge = true;

  for (const auto edge : intersections[current_edge]) {
    if (current_clique.contains(edge)) {
      continue;
    }

    if (PickCliques(cliques, current_clique, intersections, edge, k) ==
        CliqueComplementResult::kSuccess) {
      is_final_edge = false;
    }
  }

  if (is_final_edge && current_clique.size() >= k) {
    const auto hash =
        boost::hash_range(current_clique.begin(), current_clique.end());

    if (!cliques.contains(hash)) {
      cliques[hash] =
          std::make_unique<std::set<std::size_t>>(std::move(current_clique));
    }

    return CliqueComplementResult::kSuccess;
  }

  return CliqueComplementResult::kFailure;
}

}  // namespace

Graph::Graph(const std::vector<Vertex> &vertices) { AddVertices(vertices); }

Graph::Graph(std::vector<Vertex> &&vertices) {
  AddVertices(std::move(vertices));
}

Graph::Graph(std::vector<VertexSptrConst> &&vs, std::vector<EdgeSptrConst> &&es)
    : vs_(std::move(vs)), es_(std::move(es)) {}

const std::vector<VertexSptrConst> &Graph::get_vertices() const &noexcept {
  return vs_;
}

const std::vector<EdgeSptrConst> &Graph::get_edges() const &noexcept {
  return es_;
}

// TODO: rewrite adequately
GraphUptr Graph::Graphviz(const std::string &path) {
  {
    const auto cmd = "dot2tex " + path + " -ftikz -tmath -o " + path + ".tex";
    std::system(cmd.c_str());
  }

  std::ifstream tex_file(path + ".tex");

  if (!tex_file.is_open()) {
    throw std::runtime_error("Failed to open .tex file");
  }

  std::string line = "";
  while (std::getline(tex_file, line)) {
    if (line.find(kTikzpictureStart) != std::string::npos) {
      break;
    }
  }

  if (tex_file.eof()) {
    throw std::runtime_error("Failed to find the tikzpicture start");
  }

  std::unordered_map<std::string, VertexSptrConst> labels_to_vertices;
  std::vector<EdgeSptrConst> es;

  while (std::getline(tex_file, line)) {
    if (line.find(kNodeCommandStart) != std::string::npos) {
      HandleNodeCommand(std::move(line), labels_to_vertices);
    } else if (line.find(kDrawCommandStart) != std::string::npos) {
      HandleDrawCommand(std::move(line), es, labels_to_vertices);
    } else if (line.find(kTikzpictureEnd) != std::string::npos) {
      break;
    }
  }

  {
    const auto cmd = "rm " + path + ".tex";
    std::system(cmd.c_str());
  }

  auto vs = utils::ToVector(std::move(labels_to_vertices));
  return std::make_unique<Graph>(std::move(vs), std::move(es));
}

void Graph::AddVertex(const Vertex &v) {
  vs_.push_back(std::make_shared<const Vertex>(v));
}

void Graph::AddVertex(Vertex &&v) {
  vs_.push_back(std::make_shared<const Vertex>(std::move(v)));
}

void Graph::AddVertices(const std::vector<Vertex> &vs) {
  vs_.reserve(vs_.size() + vs.size());
  for (const auto &v : vs) {
    AddVertex(v);
  }
}

void Graph::AddVertices(std::vector<Vertex> &&vs) {
  vs_.reserve(vs_.size() + vs.size());
  for (auto &v : vs) {
    AddVertex(std::move(v));
  }
}

void Graph::AddSLEdge(const std::size_t start, const std::size_t end) {
  if (start >= vs_.size() || end >= vs_.size()) {
    throw std::out_of_range("Vertex index is out of rangle");
  }
  es_.push_back(std::make_shared<const Edge>(vs_[start], vs_[end]));
}

void Graph::AddSLEdges(
    const std::vector<std::pair<std::size_t, std::size_t>> &vertex_pairs) {
  for (const auto &[start, end] : vertex_pairs) {
    AddSLEdge(start, end);
  }
}

std::vector<EdgeSptrConst> Graph::CheckPlanar(const std::size_t k) const {
  if (k < 1) {
    throw std::logic_error("Graph::CheckKPlanar: failed k >= 1");
  }

  const auto intersections = CalculateIntersections();
  std::vector<EdgeSptrConst> unsatisfying_edges;

  for (std::size_t i = 0; i < es_.size(); i++) {
    if (intersections[i].size() > k) {
      unsatisfying_edges.push_back(es_[i]);
    }
  }

  return unsatisfying_edges;
}

std::vector<std::vector<EdgeSptrConst>> Graph::CheckQuasiPlanar(
    const std::size_t k) const {
  if (k < 3) {
    throw std::logic_error("Graph::CheckKQuasiPlanar: failed k >= 3");
  }

  auto intersections = CalculateIntersections();
  auto unsat_edges = kQuasiPlanarCandidates(intersections, k);

  if (unsat_edges.empty()) {
    return {};
  }

  std::unordered_map<std::size_t, std::unique_ptr<std::set<std::size_t>>>
      cliques;
  for (const auto edge : unsat_edges) {
    PickCliques(cliques, {}, intersections, edge, k);
  }

  std::vector<std::vector<EdgeSptrConst>> k_cliques;
  k_cliques.reserve(cliques.size());  // lower bound

  std::unordered_set<std::size_t> considered_k_cliques;
  considered_k_cliques.reserve(cliques.size());  // lower bound

  for (const auto &[_, clique] : cliques) {
    for (const auto &combination :
         utils::Combinations(utils::AsVector(*clique), k)) {
      const auto hash =
          boost::hash_range(combination.begin(), combination.end());
      if (!considered_k_cliques.contains(hash)) {
        considered_k_cliques.insert(hash);
        k_cliques.push_back(EdgesByIndices(combination));
      }
    }
  }

  return k_cliques;
}

std::vector<std::vector<EdgeSptrConst>> Graph::CheckSkewness(
    const std::size_t k) const {
  if (k < 1) {
    throw std::logic_error("Graph::CheckKSkewness: failed k >= 1");
  }

  if (es_.size() <= 1) {
    return {};
  }

  auto intersections = CalculateIntersections();
  std::unordered_set<std::size_t> edges_to_delete;
  int deletions_left = k;

  while (true) {
    std::size_t edge_with_most_intersections = 0;

    for (std::size_t i = 0; i < intersections.size(); i++) {
      if (intersections[i].size() >
          intersections[edge_with_most_intersections].size()) {
        edge_with_most_intersections = i;
      }
    }

    if (intersections[edge_with_most_intersections].size() == 0) {
      break;
    }

    for (const auto intersected_edge :
         intersections[edge_with_most_intersections]) {
      intersections[intersected_edge].erase(edge_with_most_intersections);
    }

    intersections[edge_with_most_intersections].clear();
    edges_to_delete.insert(edge_with_most_intersections);
    deletions_left--;
  }

  if (deletions_left >= 0) {
    return {};
  }

  const auto combinations =
      utils::Combinations(utils::AsVector(edges_to_delete), k);

  std::vector<std::vector<EdgeSptrConst>> result;
  result.reserve(combinations.size());

  for (const auto &combination : combinations) {
    result.push_back(EdgesByIndices(combination));
  }

  return result;
}

std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> Graph::CheckRAC() const {
  return CheckACE(std::numbers::pi / 2);
}

std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> Graph::CheckACE(
    const double alpha) const {
  return CheckAC([alpha](const double angle) { return angle == alpha; });
}

std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> Graph::CheckACL(
    const double alpha) const {
  return CheckAC([alpha](const double angle) { return angle >= alpha; });
}

std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> Graph::CheckAC(
    const ACPredicat &is_satisfying_angle) const {
  const auto intersections =
      CalculateIntersections(WriteIntersections::kAsymmetrically);

  std::vector<utils::Vector> directions;
  directions.reserve(es_.size());
  for (const auto &edge : es_) {
    directions.emplace_back(*edge);
  }

  std::vector<std::pair<EdgeSptrConst, EdgeSptrConst>> unsatisfying_edge_pairs;

  for (std::size_t i = 0; i < intersections.size(); i++) {
    for (const auto j : intersections[i]) {
      const auto angle = directions[i].AngleWith(directions[j]);

      assert(0 <= angle && angle <= std::numbers::pi / 2);

      if (!is_satisfying_angle(angle)) {
        unsatisfying_edge_pairs.push_back({es_[i], es_[j]});
      }
    }
  }

  return unsatisfying_edge_pairs;
}

std::vector<KLGrid> Graph::CheckGridFree(const std::size_t k,
                                         const std::size_t l) const {
  if (k == 0) {
    throw std::logic_error("Graph::CheckGridFree: failed k >= 1");
  }

  if (l == 0) {
    throw std::logic_error("Graph::CheckGridFree: failed l >= 1");
  }

  const auto intersections = CalculateIntersections<std::set<std::size_t>>();

  std::vector<std::size_t> k_groups_candidates;
  for (std::size_t i = 0; i < intersections.size(); i++) {
    if (intersections[i].size() >= l) {
      k_groups_candidates.push_back(i);
    }
  }

  std::unordered_map<std::size_t, std::unique_ptr<std::vector<std::size_t>>>
      l_groups;
  std::unordered_map<std::size_t, std::unique_ptr<std::vector<std::size_t>>>
      k_groups;

  std::unordered_map<std::size_t,
                     std::unique_ptr<std::unordered_set<std::size_t>>>
      k_l_hashes;

  for (std::size_t i = 0; i + k <= k_groups_candidates.size(); i++) {
    std::unordered_map<std::size_t, std::unique_ptr<std::vector<std::size_t>>>
        intermidiate_k_groups;
    for (std::size_t j = i + 1; j < k_groups_candidates.size(); j++) {
      const auto intersection =
          utils::Intersect(intersections[i], intersections[j]);

      if (intersection.size() < l) {
        continue;
      }

      for (auto &l_combination : utils::Combinations(intersection, l)) {
        const auto hash =
            boost::hash_range(l_combination.begin(), l_combination.end());

        if (!l_groups.contains(hash)) {
          l_groups[hash] = std::make_unique<std::vector<std::size_t>>(
              std::move(l_combination));
        }

        if (!intermidiate_k_groups.contains(hash)) {
          intermidiate_k_groups[hash] =
              std::make_unique<std::vector<std::size_t>>(
                  std::vector<std::size_t>{i, j});
        } else {
          intermidiate_k_groups[hash]->push_back(j);
        }
      }
    }

    for (const auto &[l_hash, intermidiate_k_group] : intermidiate_k_groups) {
      if (intermidiate_k_group->size() < k) {
        continue;
      }

      for (auto &k_group : utils::Combinations(*intermidiate_k_group, k)) {
        const auto k_hash = boost::hash_range(k_group.begin(), k_group.end());

        if (!k_groups.contains(k_hash)) {
          k_groups[k_hash] =
              std::make_unique<std::vector<std::size_t>>(std::move(k_group));
        }

        if (auto it = k_l_hashes.find(k_hash); it == k_l_hashes.end()) {
          k_l_hashes[k_hash] =
              std::make_unique<std::unordered_set<std::size_t>>(
                  std::unordered_set<std::size_t>{l_hash});
        } else {
          it->second->insert(l_hash);
        }
      }
    }
  }

  std::vector<KLGrid> k_l_grids;
  k_l_grids.reserve(k_l_hashes.size());  // lower bound

  for (const auto &[k_hash, l_hashes] : k_l_hashes) {
    const auto k_group = EdgesByIndices(*k_groups[k_hash]);

    for (const auto l_hash : *l_hashes) {
      k_l_grids.emplace_back(k_group, EdgesByIndices(*l_groups[l_hash]));
    }
  }

  return k_l_grids;
}

bool Graph::IsStraightLine() const {
  for (const auto &edge : es_) {
    if (!edge->IsStraightLine()) {
      return false;
    }
  }

  return true;
}

template <typename Container>
std::vector<EdgeSptrConst> Graph::EdgesByIndices(
    const Container &indices) const {
  std::vector<EdgeSptrConst> edges;
  edges.reserve(indices.size());

  for (const auto index : indices) {
    edges.push_back(es_[index]);
  }

  return edges;
}

template <typename Set>
std::vector<Set> Graph::CalculateIntersections(
    const WriteIntersections mode) const {
  std::vector<Set> intersections(es_.size());

  for (auto i = 0; i < es_.size() - 1; i++) {
    for (auto j = i + 1; j < es_.size(); j++) {
      if (es_[i]->IsIntersect(*es_[j])) {
        intersections[i].insert(j);
        if (mode == WriteIntersections::kSymmetrically) {
          intersections[j].insert(i);
        }
      }
    }
  }

  return intersections;
}

}  // namespace graph
