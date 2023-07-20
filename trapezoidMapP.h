#pragma once

#include "allocator.h"
#include "vec2.h"

#include "node.h"
#include "primitives.h"

#include <set>
#include <unordered_map>

#define GET_REAL_ID(NODE_ID) _nodes[NODE_ID].value

class TrapezoidMapP
{
public:
  void AddPolygon(const Vec2Set &points, bool compactPoints = false);
  void Build();

  struct
  {
    double tolerance       = 1e-16;
    bool checkIntersection = false;
    unsigned short phase   = 0; /* phase to update region cache of vertices */
  } config;

protected:
  // AddVertex: true - already added, false - newly added
  bool AddVertex(VertexID vertexID, NodeID startNodeID = ROOT_NODE_ID);
  bool AddSegment(SegmentID segmentID);

  RegionID Query(VertexID vertexID);
  RegionID QueryFrom(NodeID regionID, VertexID vertexID);

  VertexID AppendVertex(const Vertex &vertex);
  SegmentID AppendSegment(bool downward);

  virtual void AssignDepth();

  std::vector<SegmentID> _permutation;

  // for merging
  RegionID _nextRegion = INVALID_INDEX, _tmpRegionToMerge = INVALID_INDEX;
  int _mergeType = 0;

protected:
  Allocator<Node> _nodes;
  Allocator<Region> _regions;
  Allocator<Segment> _segments;
  Allocator<Vertex> _vertices;

  // std::unordered_map<std::pair<VertexID, VertexID>, SegmentID> _segmentMap;
  std::vector<VertexID> _endVertices, _prevVertices;

  // for vertex queries
  struct VertexNeighborInfo
  {
    RegionID left = INVALID_INDEX, mid = INVALID_INDEX, right = INVALID_INDEX;

    inline int Size() const { return Valid(left) + Valid(mid) + Valid(right); }
  };
  std::vector<VertexNeighborInfo> _lowNeighbors;
  std::vector<NodeID> _vertexRegions;  // invalid for already added

protected:
  inline static bool Valid(NodeID index) { return index != INVALID_INDEX; }
  inline Node &NewNode(Node::Type type)
  {
    Node node = _nodes.New();
    node.id   = _nodes.Size() - 1;
    node.type = type;
    return node;
  }
  template <class... Args>
  Region &NewRegion(Args... args)
  {
    Node &node     = NewNode(Node::REGION);
    Region &region = _regions.New(args);
    node.value     = _regions.Size() - 1;
    region.nodeID  = _nodes.Size() - 1;
    return region;
  }
  inline NodeID NewVertex(VertexID vertexID)
  {
    Node &node = NewNode(Node::VERTEX);
    node.value = vertexID;
    return node.id;
  }

  RegionID GetFirstIntersectedRegion(VertexID highVertex, VertexID refVertex, int *type);

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

  bool Higher(VertexID leftVertexID, VertexID rightVertexID) const;
  int Higher(const Vertex &leftVertex, const Vertex &rightVertex) const;
  bool Higher /* Lefter */ (VertexID refVertexID,
                            VertexID highVertexID,
                            VertexID lowVertexID) const;
  int Higher /* Lefter */ (const Vertex &refVertex,
                           const Vertex &highVertex,
                           const Vertex &lowVertex) const;
  bool Intersected(VertexID vertex1_1,
                   VertexID vertex1_2,
                   VertexID vertex2_1,
                   VertexID vertex2_2,
                   Vertex *const intersection) const;
  bool PointOnSegment();
};
