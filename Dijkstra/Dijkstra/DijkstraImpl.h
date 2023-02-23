#pragma once

#include "Dijkstra.h"

#include <iostream>
#include <set>
#include <cmath>
#include <chrono>

graph<char, size_t> createGraph()
{
  graph<char, size_t> g;

  g['A'] = {{'B', 6}, {'D', 1}};
  g['B'] = {{'A', 6}, {'D', 2}, {'E', 2}, {'C', 5}};
  g['D'] = {{'A', 1}, {'B', 2}, {'E', 1}};
  g['E'] = {{'D', 1}, {'B', 2}, {'C', 5}};
  g['C'] = {{'E', 5}, {'B', 5}};

  return g;
}

template <typename VertexType, typename EdgeType>
void bfs(const graph<VertexType, EdgeType>& g, const VertexType& source)
{
  if (g.find(source) == g.cend())
    return;

  std::unordered_set<VertexType> visited;

  std::queue<VertexType> bfs;
  bfs.push(source);

  while (bfs.size()) {
    VertexType front = bfs.front();
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

template <typename coordType>
struct std::hash<point<coordType>>;

template <typename weightType, typename coordType>
std::vector<point<coordType>> dijkstra(const grid<weightType>& g,
                                       const point<coordType>& source,
                                       const point<coordType>& target)
{
  using point = point<coordType>;

  if (!g.size() || !g.at(0).size())
    return {};

  auto isOnGrid = [](const grid<weightType>& g, point p) {
    return p.first >= 0 && p.first < g.size() &&
           p.second >= 0 && p.second < g.at(0).size();
  };

  if (!isOnGrid(g, source) || !isOnGrid(g, target))
    return {};

  grid<point> prev(g.size(), std::vector<point>(g.at(0).size(), point{}));
  grid<double> minWeight(g.size(), std::vector<double>(g.at(0).size(),
                                                       std::numeric_limits<double>::max()));
  minWeight[source.first][source.second] = 0.0;

  auto comp = [](const std::pair<point, double>& a, const std::pair<point, double>& b) {
    return a.second > b.second;
  };

  std::priority_queue<
  std::pair<point, double>,
  std::vector<std::pair<point, double>>,
  decltype(comp)> minHeap(comp);

  minHeap.push({source, 0.0});

  std::unordered_set<point> visited;

  auto doGreedy = [&](const point& curr, const point& next, double dist) {
    if (isOnGrid(g, next) &&
        g[next.first][next.second] < std::numeric_limits<weightType>::max()) {
      double w =
        dist * (1 + g[next.first][next.second]) + minWeight[curr.first][curr.second];

      if (w < minWeight[next.first][next.second]) {
        minWeight[next.first][next.second] = w;
        prev[next.first][next.second] = {curr.first, curr.second};
      }

      if (visited.find(next) == visited.cend())
        minHeap.push({next, minWeight[next.first][next.second]});
    }
  };

  while (minHeap.size()) {
    auto top = minHeap.top();
    minHeap.pop();

    int row = top.first.first;
    int col = top.first.second;

    if (visited.find(top.first) != visited.cend())
      continue;

    visited.insert(top.first);

    if (g[row][col] == std::numeric_limits<weightType>::max() || top.first == target)
      continue;

    doGreedy(top.first, {row - 1, col}, 1);
    doGreedy(top.first, {row + 1, col}, 1);
    doGreedy(top.first, {row, col - 1}, 1);
    doGreedy(top.first, {row, col + 1}, 1);
    doGreedy(top.first, {row - 1, col - 1}, sqrt(2));
    doGreedy(top.first, {row - 1, col + 1}, sqrt(2));
    doGreedy(top.first, {row + 1, col - 1}, sqrt(2));
    doGreedy(top.first, {row + 1, col + 1}, sqrt(2));
  }

  if (minWeight[target.first][target.second] == std::numeric_limits<double>::max())
    return {};

  std::vector<point> result;

  auto curPoint = target;
  auto prevPoint = prev[curPoint.first][curPoint.second];

  while (curPoint != source) {
    result.push_back(curPoint);

    curPoint = prevPoint;
    prevPoint = prev[prevPoint.first][prevPoint.second];
  }

  result.push_back(source);

  std::reverse(result.begin(), result.end());

  return result;
}

template <typename coordType>
struct std::hash<point<coordType>> {
  size_t operator()(const point<coordType>& p) const
  {
    auto hasher = std::hash<coordType>{};
    size_t h0 = hasher(p.first);
    size_t h1 = hasher(p.second);

    return h1 + 0x9e3779b9 + (h0<<6) + (h0>>2);
  }
};
