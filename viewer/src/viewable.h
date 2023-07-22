#pragma once

#include <functional>

#include "triangulator.h"

class ViewableTriangulator : public Triangulator
{
public:
  void Draw(Vec2 origin, Vec2 factor) const;
  void GetBoundingBox(Vec2 &leftTop, Vec2 &rightBottom) const;

  bool _needRedraw = true;

  virtual Triangles Triangulate() const;
  virtual Mountains ExtractMountains() const;

  struct
  {
    std::function<void(const Vertex &)> *vertexDrawer     = &defaultVertexDrawer;
    std::function<void(const Segment &)> *segmentDrawer   = &defaultSegmentDrawer;
    std::function<void(const Region &)> *regionDrawer     = &defaultRegionDrawer;
    std::function<void(const Mountain &)> *mountainDrawer = &defaultMountainDrawer;
    std::function<void(const Triangle &)> *triangleDrawer = &defaultTriangleDrawer;
  } methods;

protected:
  static std::function<void(const Vertex &)> defaultVertexDrawer;
  static std::function<void(const Segment &)> defaultSegmentDrawer;
  static std::function<void(const Region &)> defaultRegionDrawer;
  static std::function<void(const Mountain &)> defaultMountainDrawer;
  static std::function<void(const Triangle &)> defaultTriangleDrawer;

  Mountains _mountains;
  Triangles _triangles;
};