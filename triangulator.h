#pragma once

#include "trapezoidMapP.h"

#include <list>  // todo: memory issue
#include <map>

using Triangle  = Vec2[3];
using Triangles = std::vector<Triangle>;
using Mountain  = std::vector<VertexID>;  // first is the base
using Mountains = std::vector<Mountain>;  // first is the base

class Triangulator : public TrapezoidMapP
{
public:
  // virtual void AssignDepth() override;

  Triangles Triangulate();

  Mountains ExtractMountains();
  void ExtractCycles();

  Triangles EarClipping(const Mountain &mountain);
  Triangles ChimneyClipping(const Mountain &mountain);

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
  std::vector<bool> _vertexTransversed;

protected:
  bool IsConvex(VertexID vertices[3]);
};