#pragma once

#ifndef SEIDEL_TRIANGULATOR_H
#  define SEIDEL_TRIANGULATOR_H

#  include "trapezoidMapP.h"

#  include <list>  // todo: memory issue
#  include <map>

struct Triangle
{
  Vec2 vertices[3];

  Triangle() = default;
  Triangle(const Vertex &a, const Vertex &b, const Vertex &c) : vertices{a, b, c} {};

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
    enum CoarselyNearVerticesPolicy : int
    {
      DO_NOTHING,
      PREPROCESS,
      DYNAMIC_PROCESS,
    };

    enum ZeroSizeTrianglePolicy : int
    {
      DISCARD_ALL    = 0,
      KEEP_LINELIKE  = 0x01,
      KEEP_POINTLIKE = 0x02 | KEEP_LINELIKE,
      KEEP_ALL       = KEEP_LINELIKE | KEEP_POINTLIKE,
    };

    enum MountainResolutionPolicy : int
    {
      EAR_CLIPPING_NORMAL,  // use a stack
      EAR_CLIPPING_RANDOM,  // randomly clip ears
      EAR_CLIPPING_SORTED,  // presort ears by convexity, use a priority queue
      CHIMNEY_CLIPPING_NORMAL,
      CHIMNEY_CLIPPING_GREEDY,
    };

    double tolerance                                      = 1e-10;
    bool useNeighborCacheToTransverse                     = false;  // the second be base vertex if true
    bool multithreading                                   = false;  // todo: very problematic
    MountainResolutionPolicy mountainResolutionMethod     = CHIMNEY_CLIPPING_NORMAL;
    CoarselyNearVerticesPolicy coarselyNearVerticesPolicy = DO_NOTHING;  // todo: implement this
    ZeroSizeTrianglePolicy zeroSizeTrianglePolicy         = KEEP_ALL;
  } configTri;

protected:
  virtual Mountains ExtractMountains() const;

  void TriangulateMountainProxy(const Mountains &mountains,
                                unsigned int baseCount,
                                unsigned int count,
                                Triangles &out,
                                unsigned int baseID = -1) const;
  void TriangulateMountain(const Mountain &mountain,
                           Triangles &out,
                           bool clockwise,
                           unsigned int baseID = -1) const;

  /* 0 - normal, 1 - random, 2 - sorted */
  void EarClipping(const Mountain &mountain, Triangles &out, bool clockwise, unsigned int baseID) const;
  void EarClippingRandom(const Mountain &mountain, Triangles &out, bool clockwise, unsigned int baseID) const;
  void EarClippingSorted(const Mountain &mountain, Triangles &out, bool clockwise, unsigned int baseID) const;
  void ChimneyClipping(const Mountain &mountain, Triangles &out, bool clockwise, unsigned int baseID) const;
  void ChimneyClipping2(const Mountain &mountain, Triangles &out, bool clockwise, unsigned int baseID) const;

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
  double AngleCos(VertexID prev, VertexID current, VertexID next) const;
  double AngleCos(const Vertex &prev, const Vertex &current, const Vertex &next) const;
};

#endif SEIDEL_TRIANGULATOR_H