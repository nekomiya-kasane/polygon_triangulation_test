#pragma once

#include "allocator.h"
#include "vec2.h"

#include <set>
#include <unordered_map>

using AnyID     = unsigned int;
using NodeID    = AnyID;
using VertexID  = AnyID;
using SegmentID = AnyID;
using RegionID  = AnyID;
using Depth     = short;

constexpr unsigned int INVALID_INDEX = -1, INFINITY_INDEX = -2;

struct Node
{
  enum Type
  {
    NONE,
    VERTEX,
    SEGMENT,
  };

  NodeID id;
  Type type;

  AnyID value;

  NodeID left = INVALID_INDEX, right = INVALID_INDEX;
};

struct Region
{
  NodeID nodeID;
  VertexID high = INVALID_INDEX, low = INVALID_INDEX;
  SegmentID left = INVALID_INDEX, right = INVALID_INDEX;

  Depth depth;

  RegionID highNeighbors[2] = {INVALID_INDEX, INVALID_INDEX},
           lowNeighbors[2]  = {INVALID_INDEX, INVALID_INDEX};
};

struct Segment
{
  VertexID highVertex, lowVertex;
  bool downward;

  VertexID from() const { return downward ? highVertex : lowVertex; }
  VertexID to() const { return downward ? lowVertex : highVertex; }
};

using Vertex   = Vec2;
using NodePair = std::pair<NodeID, NodeID>;

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
                                VertexID vertexID,
                                SegmentID segmentID,
                                int type /* 0 - high, 1 - mid, 2 - low */);
  NodeID SplitSegmentByVertex(RegionID oneSideRegionID,
                              VertexID highVertexID,
                              const Vertex &intersectionVertex, bool leftIntersected); /* returns the other side ID */

  bool Higher(VertexID leftVertexID, VertexID rightVertexID) const;
  bool Higher(const Vertex &refVertex, const Vertex &highVertex,
              const Vertex &lowVertex, const Vertex * const subRefVertex = nullptr) const;
  bool Higher /* Lefter */ (VertexID refVertexID, VertexID highVertexID, VertexID lowVertexID, VertexID subRefVertexID = INVALID_INDEX) const;
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