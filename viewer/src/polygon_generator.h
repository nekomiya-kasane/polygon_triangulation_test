#pragma once

#include "vec2.h"

class PolygonGenerator
{
public:
  static Vec2Set GenerateRandomPolygon(size_t num,
                                       int xmin = 0,
                                       int xmax = 1000,
                                       int ymin = 0,
                                       int ymax = 1000);
  static Vec2Set GenerateRandomUniquePoints(size_t num,
                                            int xmin = 0,
                                            int xmax = 1000,
                                            int ymin = 0,
                                            int ymax = 1000);
};