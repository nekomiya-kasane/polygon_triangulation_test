#include <gtest/gtest.h>

#include "triangulator.h"

// Demonstrate some basic assertions.
TEST(BasicTest, DiamondClockwise)
{
  Vec2Set points = {{-1, 1}, {0, 2}, {1, 0}, {0, -2}};

  Triangulator triangulator;
  triangulator.AddPolygon(points, true);
  triangulator.Build();
  auto results = triangulator.Triangulate();
}