#include "polygon_generator.h"
#include "seidel/triangulator.h"

#include <chrono>
#include <fstream>
#include <iostream>

template <class result_t   = std::chrono::nanoseconds,
          class clock_t    = std::chrono::steady_clock,
          class duration_t = std::chrono::nanoseconds>
auto since(std::chrono::time_point<clock_t, duration_t> const &start)
{
  return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}

int main()
{
  std::chrono::high_resolution_clock timer;
  int m           = 5;
  double interval = 0;

  std::vector<std::pair<int, double>> data;

  for (int i : {100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400})
  {
    std::cout << "#points = " << i << std::endl;
    std::cout << "Generating points ... ";
    // auto points = PolygonGenerator::GenerateRandomPolygonBySculpting(std::pow(10, i), -1e5, 1e5);

    auto points = PolygonGenerator::GenerateRandomPolygon(i, -1e3, 1e3, -1e3, 1e3);

    std::fstream file(std::to_string(i) + ".txt", std::ios::ate | std::ios::out);
    for (auto &point : points[0])
    {
      file << "{" << point.x << "," << point.y << "},\n";
    }
    file.close();

    interval = 0;
    std::cout << "Triangulating ... ";
    for (int j = 0; j < m; ++j)
    {
      auto start = std::chrono::steady_clock::now();
      try
      {
        Triangulator tri;
        tri.configTri.mountainResolutionMethod = Triangulator::ConfigTri::CHIMNEY_CLIPPING_GREEDY;
        tri.AddPolygon(points[0], true);
        tri.Build();
        tri.Triangulate();
      }
      catch (...)
      {
        j--;
        continue;
      }
      interval += since(start).count() / 1e9;
    }
    interval /= m;

    std::cout << "Average: " << interval << "s\n";
    data.push_back({200 * i, interval});
  }
}