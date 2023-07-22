#pragma once

#include "trapezoidMapP.h"

#include <list>  // todo: memory issue
#include <map>

struct Triangle
{
  Vec2 vertices[3];

  inline Vec2 &operator[](unsigned short i) { return vertices[i]; }
  inline const Vec2 &operator[](unsigned short i) const { return vertices[i]; }
};
using Triangles = std::vector<Triangle>;
using Mountain  = std::vector<VertexID>;  // first is the base
using Mountains = std::vector<Mountain>;  // first is the base

class Triangulator : public TrapezoidMapP
{
public:
  // virtual void AssignDepth() override;

  virtual Triangles Triangulate() const;

  struct
  {
    bool keepZeroSizeTriangle                = true;
    bool parallelAsConvex                    = true;
    bool earClippingInsteadOfChimneyClipping = true;
    bool useNeighborCacheToTransverse        = false;  // the second be base vertex if true
    bool multithreading                      = false;  // todo: this can be paralleled
    double tolerance                         = 1e-10;
  } config;

protected:
  virtual Mountains ExtractMountains() const;

  Triangles TriangulateMountain(const Mountain &mountain, Triangles &out) const;

  Triangles EarClipping(const Mountain &mountain, Triangles &out) const;
  Triangles ChimneyClipping(const Mountain &mountain, Triangles &out) const;

  bool IsZeroSize(VertexID vertices[3]) const;
  bool IsConvex(VertexID vertices[3]) const;
};