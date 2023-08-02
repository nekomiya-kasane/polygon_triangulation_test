#pragma once

#ifndef SEIDEL_TRIANGULATOR_H
#  define SEIDEL_TRIANGULATOR_H

#  include "trapezoidMapP.h"

#  include <list>  // todo: memory issue
#  include <map>

struct Triangle
{
  Vec2 vertices[3];

  inline Vec2 &operator[](unsigned short i) { return vertices[i]; }
  inline const Vec2 &operator[](unsigned short i) const { return vertices[i]; }
};

using Triangles = std::vector<Triangle>;
using Mountain  = std::vector<VertexID>;                   // first is the base
using Mountains = std::vector<std::pair<Mountain, bool>>;  // first is the base

class Triangulator : public TrapezoidMapP
{
public:
  // virtual void AssignDepth() override;

  virtual Triangles Triangulate() const;

  struct ConfigTri
  {
    enum CoarselyNearVerticesPolicy
    {
      DO_NOTHING,
      PREPROCESS,
      DYNAMIC_PROCESS,
    };

    enum ZeroSizeTrianglePolicy
    {
      DISCARD_ALL    = 0,
      KEEP_LINELIKE  = 0x01,
      KEEP_POINTLIKE = 0x02 | KEEP_LINELIKE,
      KEEP_ALL       = KEEP_LINELIKE | KEEP_POINTLIKE,
    };

    enum EarClippingPolicy
    {
      CONVENTIONAL,  // use a stack
      RANDOM,        // randomly clip ears
      SORTED,        // presort ears by convexity, use a priority queue
    };

    enum MountainResolutionPolicy
    {
      EAR_CLIPPING,
      CHIMNEY_CLIPPING,
    };

    double tolerance                                      = 1e-10;
    bool useNeighborCacheToTransverse                     = false;  // the second be base vertex if true
    bool multithreading                                   = false;  // todo: this can be paralleled
    MountainResolutionPolicy mountainResolutionMethod     = EAR_CLIPPING;
    CoarselyNearVerticesPolicy coarselyNearVerticesPolicy = DO_NOTHING;  // todo: implement this
    ZeroSizeTrianglePolicy zeroSizeTrianglePolicy         = KEEP_ALL;
    EarClippingPolicy earClippingPolicy                   = CONVENTIONAL;
  } configTri;

protected:
  virtual Mountains ExtractMountains() const;

  void TriangulateMountain(const Mountain &mountain, Triangles &out, bool clockwise) const;

  void EarClipping(const Mountain &mountain, Triangles &out, bool clockwise) const;
  void ChimneyClipping(const Mountain &mountain, Triangles &out) const;

  bool CheckTriangle(const Triangle &triangle) const;

  bool IsZeroSize(VertexID prevID, VertexID currentID, VertexID nextID) const;
  bool IsZeroSize(const Vertex &prev, const Vertex &current, const Vertex &next) const;
  bool IsPointLike(VertexID prev, VertexID current, VertexID next, bool assumeZeroSize = false) const;
  bool IsPointLike(const Vertex &prev,
                   const Vertex &current,
                   const Vertex &next,
                   bool assumeZeroSize = false) const;
  bool IsConvex(VertexID prev, VertexID current, VertexID next, bool clockwise = false) const;
  bool IsConvex(const Vertex &prev, const Vertex &current, const Vertex &next, bool clockwise = false) const;
};

#endif SEIDEL_TRIANGULATOR_H