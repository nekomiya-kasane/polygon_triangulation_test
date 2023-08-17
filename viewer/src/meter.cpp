#include "fist/triangulator.h"
#include "polygon_generator.h"
// #include "seidel/triangulator.h"

#include <chrono>
#include <fstream>
#include <iostream>

int main()
{
  std::chrono::high_resolution_clock timer;
  auto start      = timer.now();
  double interval = 0;
  int m           = 1;

  std::vector<std::pair<int, double>> data;

  for (int i : {100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400})
  {
    interval = 0;

    std::cout << "#points = " << i << std::endl;
    std::cout << "Generating points ... ";
    // auto points = PolygonGenerator::GenerateRandomPolygonBySculpting(std::pow(10, i), -1e5, 1e5);
    for (int j = 0; j < m; ++j)
    {
      std::cout << j << " ";
      start       = timer.now();
      auto points = PolygonGenerator::GenerateRandomPolygon(i, -1e3, 1e3, -1e3, 1e3);
      interval    = (timer.now() - start).count() / 1e9;

      std::fstream file(std::to_string(i) + ".txt", std::ios::ate | std::ios::out);
      for (auto &point : points[0])
      {
        file << "{" << point.x << "," << point.y << "},\n";
      }
      file.close();

      start = timer.now();
      try
      {
        // Triangulator tri;
        tri.configTri.mountainResolutionMethod = Triangulator::ConfigTri::CHIMNEY_CLIPPING_GREEDY;
        // tri.AddPolygon(points[0], true);
        // tri.config.useGivenSeed = true;
        // tri.Build();
        // tri.Triangulate();

        FistTriangulator tri(i);
        tri.SetBoundary((double *)points[0].data(), i);
        tri.Triangulate();

        interval = (timer.now() - start).count() / 1e9;
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