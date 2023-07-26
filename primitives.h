#pragma once

#include <utility>

#include "vec2.h"

#ifdef SEIDEL_USE_SIGNED_INDEX
using AnyID = int;
#else
using AnyID = unsigned int;
#endif
using NodeID    = AnyID;
using VertexID  = AnyID;
using SegmentID = AnyID;
using RegionID  = AnyID;
using Depth     = short;

constexpr AnyID INVALID_INDEX = -1, INFINITY_INDEX = -2, ROOT_REGION_ID = 0, ROOT_NODE_ID = 0;
constexpr Depth INVALID_DEPTH = -1;

struct Node
{
  enum Type
  {
    NONE    = 0,
    VERTEX  = 1,
    SEGMENT = 2,
    REGION  = 3
  };

  NodeID id;
  Type type;

  AnyID value;

  NodeID left = INVALID_INDEX, right = INVALID_INDEX;
};

struct Region
{
  Region();

  NodeID nodeID;
  VertexID high, low;
  SegmentID left, right;

  Depth depth;

  RegionID highNeighbors[2], lowNeighbors[2];
};

struct Segment
{
  // NodeID nodeID;
  /* don't move this line */ VertexID highVertex, lowVertex;
  /* don't move this line */ bool downward;

  inline const VertexID &from() const { return downward ? highVertex : lowVertex; }
  inline const VertexID &to() const { return downward ? lowVertex : highVertex; }
  inline VertexID &from() { return downward ? highVertex : lowVertex; }
  inline VertexID &to() { return downward ? lowVertex : highVertex; }
};

using Vertex   = Vec2;
using NodePair = std::pair<NodeID, NodeID>;