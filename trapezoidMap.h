#pragma once

#include "allocator.h"
#include "primitives.h"

#include <set>
#include <unordered_map>

class TrapezoidMap
{
public:
  void Build();

  // AddVertex: true - already added, false - newly added
  bool AddVertex(VertexID vertexID);
  bool AddSegment(VertexID from, VertexID to);

  RegionID Query(const Vertex &point);
  RegionID QueryFrom(RegionID region, const Vertex &point);

protected:
  Allocator<Node> _nodes;
  Allocator<Region> _regions;
  Allocator<Segment> _segments;
  Allocator<Vertex> _vertices;

  std::unordered_map<std::pair<VertexID, VertexID>, SegmentID> _segmentMap;
  std::vector<NodeID> _vertexAdded;

  struct VertexNeighborInfo
  {
    NodeID regionID;
    double leftAngle, rightAngle;
  };
  struct VertexNeighborInfoComparator
  {
    bool operator()(const VertexNeighborInfo &left, const VertexNeighborInfo &right) const;
  };
  std::vector<std::set<VertexNeighborInfo, VertexNeighborInfoComparator>> _highNeighbors,
      _lowNeighbors;  // for vertices

protected:
  inline bool Valid(NodeID index) const { return index != INVALID_INDEX; }
  Node &NewNode(Node::Type type)
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
  NodeID NewVertex(VertexID vertexID)
  {
    Node &node = NewNode(Node::VERTEX);
    node.value = vertexID;
    return node.id;
  }
  NodeID NewSegment(SegmentID segmentID)
  {
    Node &node = NewNode(Node::SEGMENT);
    node.value = segmentID;
    return node.id;
  }

  RegionID GetRegionNext(VertexID highVertex, VertexID refVertex);
  RegionID GetRegionNext(RegionID thisRegionID, VertexID highVertex, VertexID lowVertex);

  NodePair SplitRegionByVertex(RegionID regionID, VertexID vertexID);
  NodePair SplitRegionBySegment(RegionID regionID,
                                SegmentID segmentID,
                                VertexID segmentHighVertexID,
                                VertexID segmentLowVertexID,
                                int type /* 0 - high, 1 - mid, 2 - low */);
  NodeID SplitSegmentByVertex(RegionID oneSideRegionID,
                              VertexID highVertexID,

                              const Vertex &intersectionVertex,
                              bool leftIntersected); /* returns the other side ID */

  bool Higher(VertexID leftVertexID, VertexID rightVertexID) const;
  int Higher(const Vertex &leftVertex, const Vertex &rightVertex) const;
  bool Higher /* Lefter */ (VertexID refVertexID,
                            VertexID highVertexID,
                            VertexID lowVertexID,
                            VertexID subRefVertexID = INVALID_INDEX) const;
  int Higher /* Lefter */ (const Vertex &refVertex,
                           const Vertex &highVertex,
                           const Vertex &lowVertex,
                           const Vertex *const subRefVertexPtr = nullptr) const;
  bool Intersected(SegmentID segment,
                   VertexID highVertex,
                   VertexID lowVertex,
                   Vertex *const intersection) const;
  bool InInterval(double value,
                  double low,
                  double high       = 0,
                  bool lowIncluded  = false,
                  bool highIncluded = false);
};