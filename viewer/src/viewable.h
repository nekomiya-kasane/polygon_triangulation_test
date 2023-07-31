#pragma once

#include <functional>

#include "triangulator.h"

#ifndef USE_EASYX
#  include "raylib.h"
#endif

class ViewableTriangulator : public Triangulator
{
public:
  ~ViewableTriangulator();

  void SetOrigin(Vec2 origin);
  void Draw(Vec2 origin, Vec2 factor);
  void GetBoundingBox(Vec2 &leftTop, Vec2 &rightBottom) const;

  bool _needRedraw = true;

  virtual Triangles Triangulate() const;
  virtual Mountains ExtractMountains() const;

  using VertexDrawer   = std::function<void(const Vertex &, const std::string &)>;
  using SegmentDrawer  = std::function<void(const Segment &, const std::string &)>;
  using RegionDrawer   = std::function<void(const Region &, const std::string &)>;
  using MountainDrawer = std::function<void(const Mountain &, const std::string &)>;
  using TriangleDrawer = std::function<void(const Triangle &, const std::string &)>;

  struct
  {
    VertexDrawer *vertexDrawer     = nullptr;
    SegmentDrawer *segmentDrawer   = nullptr;
    RegionDrawer *regionDrawer     = nullptr;
    MountainDrawer *mountainDrawer = nullptr;
    TriangleDrawer *triangleDrawer = nullptr;
  } methods;

  struct
  {
    int vertexRadius = 4;
#ifndef USE_EASYX
    Font font;
#endif
  } drawingConfig;

protected:
  Mountains _mountains;
  Triangles _triangles;

  Vec2 _centroid, _origin, _factor;

#ifdef USE_EASYX
  inline int x(double ix) const { return static_cast<int>((ix - _centroid.x) * _factor.x + _origin.x); }
  inline int y(double iy) const { return static_cast<int>((iy - _centroid.y) * _factor.y + _origin.y); }
  int evalX(double iy, const Segment &seg) const;
#else
  inline float x(double ix) const { return (ix - _centroid.x) * _factor.x + _origin.x; }
  inline float y(double iy) const { return (iy - _centroid.y) * _factor.y + _origin.y; }
  float evalX(double iy, const Segment &seg) const;
#endif
};