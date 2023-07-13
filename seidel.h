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
  NodeIndex Query(const Vec2 &point);
  NodeIndex QueryFrom(NodeIndex nodeID, const Vec2 &point);

  // tree
  NodeIndex AddVectex(const Vec2 &pointLoc, Index pointID);
  SegmentNode *AddSegment(Index fromID, Index toID);

  [[nodiscard]] NodeIndex SplitRegionByPoint(NodeIndex regionID, Index pointID);
  [[nodiscard]] RegionNode *SplitRegionByEdge(NodeIndex regionID,
                                              NodeIndex highVertexID,
                                              NodeIndex lowVertexID, bool downward, int catagory);

  NodeIndex GetVertexRegion(NodeIndex id);
  void UpdateVertexPosition();

  NodeIndex FindRegionBelow(NodeIndex regionID, NodeIndex highVertexID, NodeIndex lowVertexID) const;
  NodeIndex FindRegionBelow(NodeIndex vertexID, const Vec2 &ref, int *catagory) const;
  void UpdateFirstSplitRegion(RegionNode *region);
  void UpdateMiddleSplitRegion(RegionNode *region);
  void UpdateLastSplitRegion(RegionNode *region);
  void UpdateRegionsAbovePoints(RegionNode *region);

  Triangles TriangulateMonoMountain(const Indexes &vertices);
  void AssignDepth();

  void UpdatePermutation();
  static bool GenerateRandomBool(void *seed);
  bool VertexHigher(const Vec2 &left, const Vec2 &right) const;
  bool VertexLefter(const Vec2 &point, const Vec2 &lowEnd, const Vec2 &highEnd) const;
  bool SegmentIntersected(const Vec2 &from1, const Vec2 &to1, const Vec2 &from2, const Vec2 &to2, Vec2 *const intersection) const;

  // configuration
  bool _compactPoints     = false; /* false: vertex[0] same as vertex[-1] */
  bool _checkIntersection = false; /* detect intersection of segments */
  unsigned short _phase   = 0;     /* phase to update region cache of vertices */
  double _tol             = 1e-10; /* tolerance */
  void RefinePhase();

  // input
  Vec2Set _points;
  std::vector<std::pair<Index, Index>> _permutation;

  // results
  Triangles _results;

  // variables
  NodeIndexes _vertices, _segments; /* same count as points */
  size_t _vertexCount = 0, _segmentCount = 0;
  NodeIndexes _endIndices;

  struct BelowPolygonInfo
  {
    NodeIndex left = INVALID_INDEX, middle = INVALID_INDEX, right = INVALID_INDEX;

    inline size_t Count() const
    {
      return left != INVALID_INDEX + middle != INVALID_INDEX + right != INVALID_INDEX;
    }
  };
  std::vector<BelowPolygonInfo> _belowInfo;

  std::vector<bool> _vertexFlag;
  std::unordered_map<NodeIndex, NodeIndex> _vertexTreePosition;  // is

  // allocator
  NodeAllocator _alloc;
};