#include <iostream>

#include "Dijkstra.h"

int main()
{
  // auto result = dijkstra(createGraph(), 'A', 'C');

  // if (result.has_value())
  //   std::cout << "Distance: " << result.value() << std::endl;
  // else
  //   std::cout << "Target vertex isn't reachable" << std::endl;

  grid<unsigned char> g{{0, 0, 0,   0,   0,   0,   0,   0,   0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   0,   0, 0},
                        {0, 0, 255, 255, 255, 255, 255, 255, 0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   255, 0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   255, 0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   0,   0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   0,   0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   0,   0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   0,   0, 0},
                        {0, 0, 0,   0,   0,   0,   0,   0,   0, 0}};

  for (auto& v : g) {
    for (int i : v)
      std::cout << i << '\t';
    std::cout << std::endl;
  }

  std::cout << std::endl;

  using point = point<int>;
  auto result = dijkstra(g, point{7, 1}, point{1, 8});

  size_t counter = 0;
  for (auto& i : result)
    g[i.first][i.second] = ++counter;

  for (auto& v : g) {
    for (int i : v)
      std::cout << i << '\t';
    std::cout << std::endl;
  }

  return 0;
}
