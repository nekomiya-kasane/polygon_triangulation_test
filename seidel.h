#pragma once

#include "node_allocator.h"


struct Triangle
{
  Index A, B, C;
};

using Triangles = std::vector<Triangle>;


class Triangulator
{
public:
  void AddPolygon(const Vec2Set &vertices);
  const Triangles &Triangulate();

protected:
  void BuildQueryTree();

  // query
  RegionNode *Query(const Vec2 &point);
  NodeIndex QueryFrom(NodeIndex nodeID, const Vec2 &point);

  // tree
  NodeIndex AddVectex(const Vec2 &pointLoc, Index pointID);
  SegmentNode *AddSegment(Index fromID, Index toID);

  [[nodiscard]] NodeIndex SplitRegionByPoint(NodeIndex regionID, Index pointID);
  [[nodiscard]] RegionNode *SplitRegionByEdge(RegionNode *region,
                                              const VertexNode *topVertex,
                                              const VertexNode *bottomVertex);

  NodeIndex &GetVertexRegion(NodeIndex id);
  void UpdateVertexPosition();

  RegionNode *FindRegionBelow(RegionNode *region) const;
  void UpdateFirstSplitRegion(RegionNode *region);
  void UpdateMiddleSplitRegion(RegionNode *region);
  void UpdateLastSplitRegion(RegionNode *region);
  void UpdateRegionsAbovePoints(RegionNode *region);

  Triangles TriangulateMonoMountain(const Indexes &vertices);
  void AssignDepth();

  void UpdatePermutation();
  static bool GenerateRandomBool(void *seed);
  bool VertexHigher(const Vec2 &left, const Vec2 &right);
  bool VertexLefter(const Vec2 &point, const Vec2 &lowEnd, const Vec2 &highEnd);

  // configuration
  bool _compactPoints     = false; /* false: vertex[0] same as vertex[-1] */
  bool _checkIntersection = false; /* detect intersection of segments */
  unsigned short _phase   = 800;   /* phase to update region cache of vertices */
  double _tol             = 1e-10; /* tolerance */

  // input
  Vec2Set _points;
  std::vector<std::pair<Index, Index>> _permutation;

  // results
  Triangles _results;

  // variables
  NodeIndexes _vertices, _segments; /* same count as points */

  size_t _vertexCount = 0, _segmentCount = 0;

  std::vector<bool> _vertexFlag;
  std::unordered_map<NodeIndex, NodeIndex> _vertexTreePosition;  // is

  // allocator
  NodeAllocator _alloc;
};