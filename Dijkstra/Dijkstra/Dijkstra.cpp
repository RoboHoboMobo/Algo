#include "DijkstraImpl.h"

template
void bfs(const graph<char, size_t>& g, const char& source);

template
std::optional<size_t> dijkstra(const graph<char, size_t>& g,
                               const char& source, const char& target);
template
std::vector<point<int>> dijkstra(const grid<unsigned char>& g,
                                       const point<int>& source,
                                       const point<int>& target);
