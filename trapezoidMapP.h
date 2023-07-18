#pragma once

#include "allocator.h"
#include "vec2.h"

#include "node.h"
#include "primitives.h"

#include <set>
#include <unordered_map>

class TrapezoidMapP
{
public:
  void Build();

  // AddVertex: true - already added, false - newly added
  bool AddVertex(VertexID vertexID);
  bool AddSegment(VertexID from, VertexID to);

  RegionID Query(const Vertex &point);
  RegionID QueryFrom(RegionID region, const Vertex &point);

  VertexID AppendVertex(const Vertex &vertex);

protected:
  Allocator<Node> _nodes;
  Allocator<Region> _regions;
  Allocator<Vertex> _vertices;

  // std::unordered_map<std::pair<VertexID, VertexID>, SegmentID> _segmentMap;
  std::vector<VertexID> _endVertices, _prevVertices;
  std::vector<SegmentID> _segments;
  std::vector<NodeID> _vertexAdded;
  std::vector<bool> _segmentDownward;  // highVertex started segment go upward?

  struct VertexNeighborInfo
  {
    RegionID left = INVALID_INDEX, mid = INVALID_INDEX, right = INVALID_INDEX;

    inline int Size() const { return Valid(left) + Valid(mid) + Valid(right); }
  };
  std::vector<VertexNeighborInfo> _highNeighbors,
      _lowNeighbors;  // for vertices

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

  RegionID GetRegionNext(VertexID highVertex, VertexID refVertex);
  RegionID GetRegionNext(RegionID thisRegionID, VertexID highVertex, VertexID lowVertex);

  void ResolvePossibleIntersectionL();
  void ResolvePossibleIntersectionR();
  void ResolvePossibleIntersectionM();
  void ResolveIntersection(RegionID curRegionID,
                           VertexID highVertexID,
                           VertexID lowVertexID,
                           bool checkLeft,
                           bool checkRight);
  // \_     /
  // |\    /
  //   \  /
  //    \/

  void RefineCurrent(RegionID leftRegionID,
                     RegionID rightRegionID,
                     SegmentID oldSegmentID,
                     SegmentID newSegmentID,
                     const Vertex &intersection,
                     int type);

  NodePair SplitRegionByVertex(RegionID regionID, VertexID vertexID);
  NodePair SplitRegionBySegment(RegionID regionID,
                                SegmentID segmentID,
                                VertexID segmentHighVertexID,
                                VertexID segmentLowVertexID,
                                int type /* 0 - high, 1 - mid, 2 - low */);
  NodeID SplitSegmentByVertex(RegionID lastRegionID,
                              VertexID highVertexID,

                              const Vertex &intersectionVertex,
                              bool leftIntersected); /* returns the other side ID */

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
  bool InInterval(double value,
                  double low,
                  double high       = 0,
                  bool lowIncluded  = false,
                  bool highIncluded = false);
  bool PointOnSegment();

  RegionID _nextRegion = INVALID_INDEX;
};