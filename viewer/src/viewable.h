#pragma once

#include <functional>

#include "seidel/triangulator.h"

#ifndef USE_EASYX
#  pragma warning(push, 0)
#  include "raylib.h"
#  pragma warning(pop)
#endif

class ViewableTriangulator : public Triangulator
{
public:
  ~ViewableTriangulator();

  void SetOrigin(Vec2 origin);
  void SetBox(Vec2 box);
  void SetFocus(Vec2 focus);
  void SetZoomFactor(float factor);

  void Draw(Vec2 origin, Vec2 factor);
  void GetBoundingBox(Vec2 &leftTop, Vec2 &rightBottom) const;

  std::string _infoBuf;
  bool _needRedraw = true;

  virtual Triangles Triangulate() const;
  virtual Mountains ExtractMountains() const;

  using VertexDrawer =
      std::function<void(const Vertex &, const std::string &, Color color, bool overrideColor)>;
  using SegmentDrawer =
      std::function<void(const Segment &, const std::string &, Color color, bool overrideColor)>;
  using RegionDrawer =
      std::function<void(const Region &, const std::string &, Color color, bool overrideColor)>;
  using MountainDrawer =
      std::function<void(const Mountain &, const std::string &, Color color, bool overrideColor)>;
  using TriangleDrawer =
      std::function<void(const Triangle &, const std::string &, Color color, bool overrideColor)>;

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
    VertexID curVertexID   = INVALID_INDEX;
    SegmentID curSegmentID = INVALID_INDEX;
    RegionID curRegionID   = INVALID_INDEX;
  } indicators;

  struct
  {
    float vertexRadius = 4;
    Color color;
#ifndef USE_EASYX
    Font font;
#endif
  } drawingConfig;

protected:
  Mountains _mountains;
  Triangles _triangles;

  Vec2 _centroid, _origin, _factor, _box, _focus;
  float _zoom = 1.f;

#ifdef USE_EASYX
  inline int x(double ix) const { return static_cast<int>((ix - _centroid.x) * _factor.x + _origin.x); }
  inline int y(double iy) const { return static_cast<int>((iy - _centroid.y) * _factor.y + _origin.y); }
  int evalX(double iy, const Segment &seg) const;
#else
  inline float x(double ix) const { return static_cast<float>((ix - _centroid.x) * _factor.x + _origin.x); }
  inline float y(double iy) const { return static_cast<float>((iy - _centroid.y) * _factor.y + _origin.y); }
  float evalX(double iy, const Segment &seg) const;
  float evalX(double iy, const Vec2 &low, const Vec2 &high) const;
#endif
};