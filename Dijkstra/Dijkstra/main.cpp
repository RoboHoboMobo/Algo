#include <iostream>

#include <utility>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <functional>

using vertex = char;
using edge = size_t;

template <typename VertexType, typename EdgeType>
using graph = std::unordered_map<VertexType, std::vector<std::pair<VertexType, EdgeType>>>;

graph<vertex, edge> createGraph()
{
  graph<vertex, edge> g;

  g['A'] = {{'B', 6}, {'D', 1}};
  g['B'] = {{'A', 6}, {'D', 2}, {'E', 2}, {'C', 5}};
  g['D'] = {{'A', 1}, {'B', 2}, {'E', 1}};
  g['E'] = {{'D', 1}, {'B', 2}, {'C', 5}};
  g['C'] = {{'E', 5}, {'B', 5}};

  return g;
}

void bfs(const graph<vertex, edge>& g, const vertex& source)
{
  if (g.find(source) == g.cend())
    return;

  std::unordered_set<vertex> visited;

  std::queue<vertex> bfs;
  bfs.push(source);

  while (bfs.size()) {
    vertex front = bfs.front();
    bfs.pop();

    if (visited.find(front) != visited.end())
      continue;

    std::cout << front << std::endl;

    visited.insert(front);

    for (auto& [v, e] : g.at(front)) {
      if (visited.find(v) == visited.end())
        bfs.push(v);
    }
  }
}

template <typename VertexType, typename EdgeType>
std::optional<EdgeType> dijkstra(const graph<VertexType, EdgeType>& g,
                                 const VertexType& source, const VertexType& target)
{
  if (g.find(source) == g.cend() || g.find(target) == g.cend())
    return {};

  std::unordered_map<VertexType, EdgeType> minDist;

  for (const auto& [v, adj] : g)
    minDist[v] = std::numeric_limits<EdgeType>::max();

  minDist[source] = 0;

  std::unordered_map<VertexType, VertexType> prev;
  std::unordered_set<VertexType> visited;

  using vertexEdge = std::pair<VertexType, EdgeType>;

  auto compare = [](const vertexEdge& a, const vertexEdge& b) {
    return a.second > b.second;
  };

  std::priority_queue<vertexEdge, std::vector<vertexEdge>, decltype(compare)>
  minHeap{compare};

  minHeap.push({source, minDist[source]});

  while (minHeap.size()) {
    vertexEdge top = minHeap.top();
    minHeap.pop();

    if (visited.find(top.first) != visited.end())
      continue;

    visited.insert(top.first);

    if (top.first == target)
      continue;

    for (const auto& adj : g.at(top.first)) {
      EdgeType dist = adj.second + minDist[top.first];

      if (dist < minDist[adj.first]) {
        minDist[adj.first] = dist;
        prev[adj.first] = top.first;
      }

      if (visited.find(adj.first) == visited.cend())
        minHeap.push({adj.first, minDist[adj.first]});
    }
  }

  return minDist[target] < std::numeric_limits<EdgeType>::max() ?
    minDist[target] : std::optional<EdgeType>{};
}

int main()
{
  auto result = dijkstra(createGraph(), 'A', 'C');

  if (result.has_value())
    std::cout << "Distance: " << result.value() << std::endl;
  else
    std::cout << "Target vertex isn't reachable" << std::endl;

  return 0;
}
