#include "plasma/TrapezoidMap.hpp"

int main()
{
  std::vector<Vec2> points = {{{0, 0},
                               {91, 41},
                               {67, 74},
                               {0, 0},
                               {-10, 99},
                               {-50, 87},
                               {0, 0},
                               {-98, 21},
                               {-98, -21},
                               {0, 0},
                               {-50, -87},
                               {-10, -99},
                               {0, 0},
                               {67, -74},
                               {91, -41}}};
  Plasma::TrapezoidMap map(points, {8}, -1, 0);
  return 0;
}