#pragma once

#include <utility>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <functional>

template <typename VertexType, typename EdgeType>
using graph = std::unordered_map<VertexType, std::vector<std::pair<VertexType, EdgeType>>>;

graph<char, size_t> createGraph();

template <typename VertexType, typename EdgeType>
void bfs(const graph<VertexType, EdgeType>& g, const VertexType& source);

template <typename VertexType, typename EdgeType>
std::optional<EdgeType> dijkstra(const graph<VertexType, EdgeType>& g,
                                 const VertexType& source, const VertexType& target);

template <typename T>
using grid = std::vector<std::vector<T>>;

template <typename coordType>
using point = std::pair<coordType, coordType>;

template <typename weightType, typename coordType = int>
std::vector<point<coordType>> dijkstra(const grid<weightType>& g,
                                       const point<coordType>& source,
                                       const point<coordType>& target);
