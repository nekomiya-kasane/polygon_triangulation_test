#pragma once

#include <functional>

#include "triangulator.h"

class ViewableTriangulator : public Triangulator
{
public:
  void Draw(Vec2 origin, Vec2 factor);
  void GetBoundingBox(Vec2 &leftTop, Vec2 &rightBottom) const;

  bool _needRedraw = true;

  virtual Triangles Triangulate() const;
  virtual Mountains ExtractMountains() const;

  using VertexDrawer   = std::function<void(const Vertex &)>;
  using SegmentDrawer  = std::function<void(const Segment &)>;
  using RegionDrawer   = std::function<void(const Region &)>;
  using MountainDrawer = std::function<void(const Mountain &)>;
  using TriangleDrawer = std::function<void(const Triangle &)>;

  struct A
  {
    VertexDrawer *vertexDrawer     = nullptr;
    SegmentDrawer *segmentDrawer   = nullptr;
    RegionDrawer *regionDrawer     = nullptr;
    MountainDrawer *mountainDrawer = nullptr;
    TriangleDrawer *triangleDrawer = nullptr;
  } methods;

  struct
  {
    int vertexRadius = 2;
  } drawingConfig;

protected:
  Mountains _mountains;
  Triangles _triangles;
};