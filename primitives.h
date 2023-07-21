#pragma once

#include <utility>

#include "vec2.h"

using AnyID     = unsigned int;
using NodeID    = AnyID;
using VertexID  = AnyID;
using SegmentID = AnyID;
using RegionID  = AnyID;
using Depth     = short;

constexpr unsigned int INVALID_INDEX = -1, INFINITY_INDEX = -2, ROOT_REGION_ID = 0,
                       ROOT_NODE_ID = 0, INVALID_DEPTH = -1;

struct Node
{
  enum Type
  {
    NONE,
    VERTEX,
    SEGMENT,
    REGION
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
  SegmentID left = INVALID_INDEX, right = INVALID_INDEX;  // [start] Vertex of segments

  Depth depth;

  RegionID highNeighbors[2] = {INVALID_INDEX, INVALID_INDEX},
           lowNeighbors[2]  = {INVALID_INDEX, INVALID_INDEX};
};

struct Segment
{
  // NodeID nodeID;
  /* don't move this line */ VertexID highVertex, lowVertex;
  /* don't move this line */ bool downward;

  const VertexID &from() const { return downward ? highVertex : lowVertex; }
  const VertexID &to() const { return downward ? lowVertex : highVertex; }
  VertexID &from() { return downward ? highVertex : lowVertex; }
  VertexID &to() { return downward ? lowVertex : highVertex; }
};

using Vertex   = Vec2;
using NodePair = std::pair<NodeID, NodeID>;