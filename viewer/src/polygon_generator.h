#pragma once

#include "fist/vec2.h"

#include <list>

class PolygonGenerator
{
public:
  static std::vector<Vec2Set> GenerateRandomPolygon(size_t num,
                                                    int xmin = 0,
                                                    int xmax = 1000,
                                                    int ymin = 0,
                                                    int ymax = 1000);
  static Vec2Set GenerateRandomUniquePoints(size_t num,
                                            int xmin = 0,
                                            int xmax = 1000,
                                            int ymin = 0,
                                            int ymax = 1000);
  static std::list<Vec2> GetHull(const Vec2Set &iPoints, Vec2Set &oRemained);
  static std::list<Vec2> &SculpPolygon(std::list<Vec2> &ioPolygon, Vec2Set &iPoints, size_t iDepth);
  static std::vector<Vec2Set> GenerateRandomPolygonBySculpting(size_t num,
                                                               int xmin      = 0,
                                                               int xmax      = 1000,
                                                               int ymin      = 0,
                                                               int ymax      = 1000,
                                                               bool withHole = true);
};