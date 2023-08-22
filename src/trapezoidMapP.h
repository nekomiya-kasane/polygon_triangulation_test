#pragma once

#ifndef SEIDEL_TRAPEZOIDMAPP_H
#  define SEIDEL_TRAPEZOIDMAPP_H

#  include "allocator.h"
#  include "vec2.h"

#  include "primitives.h"

#  include <set>
#  include <unordered_map>

#  ifdef SEIDEL_INTERNAL_TEST
#    define SEIDEL_PRIVATE public
#  else
#    define SEIDEL_PRIVATE protected
#  endif

class TrapezoidMapP
{
public:
  void AddPolygon(const Vec2Set &points, bool compactPoints = false);
  void Build();

  void Reset();
  void ClearCache();

  struct Config
  {
    double tolerance       = 1e-16;
    bool checkIntersection = false;
    bool useGivenSeed      = false;
    int seed               = 1;
    unsigned short phase   = 0; /* phase to update `_vertexRegions` */

#  ifdef _DEBUG
    // for debugging
    bool incremental        = false;
    bool printData          = true;
    bool printCase          = false;
    bool assignDepth        = true;
    bool generateMountains  = true;
    bool triangulation      = true;
    unsigned int maxSegment = 9999;
#  endif

  } config;

  // clang-format off
SEIDEL_PRIVATE:
  /* AddVertex : true - already added, false - newly added */
  bool AddVertex(VertexID vertexID, NodeID startNodeID = ROOT_NODE_ID);
  bool AddSegment(SegmentID segmentID);
  // clang-format on

  RegionID Query(VertexID vertexID);
  RegionID QueryFrom(NodeID regionID, VertexID vertexID);

  VertexID AppendVertex(const Vertex &vertex);
  SegmentID AppendSegment(bool downward);

  void AssignDepth();

  std::vector<SegmentID> _permutation;

  // for merging
  RegionID _nextRegion = INVALID_INDEX, _tmpRegionToMerge = INVALID_INDEX;
  int _mergeType = 0; /* -1: merge left region, 0: no region to merge, 1: merge right region */

  // clang-format off
SEIDEL_PRIVATE:
  Allocator<Node> _nodes;
  Allocator<Region> _regions;
  Allocator<Segment> _segments;
  Allocator<Vertex> _vertices;
  // clang-format on

  // std::unordered_map<std::pair<VertexID, VertexID>, SegmentID> _segmentMap;
  AnyID _polygonCount = 0, _vertexCount = 0;
  Allocator<VertexID> _endVertices, _prevVertices, _endDirVertices, _prevDirVertices, _polygonIDs;

  /// for vertex queries
  struct VertexNeighborInfo
  {
    /* We define that, if there's only one region below, then it's stored in `left`; if there're 2 regions
     * below, they're stored in `left` and `right` */
    RegionID left = INVALID_INDEX, mid = INVALID_INDEX, right = INVALID_INDEX;

    inline int Size() const { return Valid(left) + Valid(mid) + Valid(right); }
  };
  /* regions right below a vertex, 3 at most for polygons */
  std::vector<VertexNeighborInfo> _lowNeighbors;
  /* Initialized to the root node. INVALID_INDEX for already added vertices. */
  std::vector<NodeID> _vertexRegions;

  // clang-format off
SEIDEL_PRIVATE:
  inline static bool Valid(AnyID index) { return index != INVALID_INDEX; }
  inline static bool Infinite(AnyID index) { return index == INFINITY_INDEX; }
  // clang-format on

  // memory allocating
  inline Node &NewNode(Node::Type type)
  {
    Node &node = _nodes.New();
    node.id    = _nodes.Size() - 1;
    node.type  = type;
    return node;
  }
  inline Node &NewNode(Node::Type type, AnyID value)
  {
    Node &node = _nodes.New();
    node.id    = _nodes.Size() - 1;
    node.type  = type;
    node.value = value;
    return node;
  }
  template <class... Args>
  Region &NewRegion(Args... args)
  {
    Node &node     = NewNode(Node::REGION, _regions.Size());
    Region &region = _regions.New(args...);
    region.nodeID  = node.id;
    return region;
  }
  inline NodeID NewVertex(VertexID vertexID)
  {
    Node &node = NewNode(Node::VERTEX);
    node.value = vertexID;
    return node.id;
  }

  // neighbors maintaining
  RegionID GetFirstIntersectedRegion(VertexID highVertex, VertexID refVertex, int *type) const;

  SegmentID ResolveIntersection(RegionID curRegionID,
                                SegmentID newSegmentID,
                                bool checkLeft,
                                bool checkRight);

  NodePair SplitRegionByVertex(RegionID regionID, VertexID vertexID);
  void SplitRegionBySegment(RegionID regionID,
                            SegmentID segmentID,
                            int &type /* 0 - high, 1 - mid, 2 - low */);

  void UpdateAbove(Region &originalRegionID,
                   Region &highRegionID,
                   Region &lowRegionID,
                   SegmentID segmentID,
                   int type);

  int UpdateBelow(RegionID originalRegionID,
                  RegionID highRegionID,
                  RegionID lowRegionID,
                  SegmentID segmentID);

  // geometric calculation
  bool Higher(VertexID leftVertexID, VertexID rightVertexID) const;
  bool Lefter(VertexID refVertexID, SegmentID segmentID) const;
  int Intersected(VertexID segment1_Start,
                  VertexID segment1_End,
                  VertexID segment2_Start,
                  VertexID segment2_End,
                  Vertex *const intersection) const;
  // bool PointOnSegment();  // todo: "T" type coincidence
};

#endif SEIDEL_TRAPEZOIDMAPP_H
