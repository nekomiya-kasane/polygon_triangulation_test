#pragma once

#ifndef SEIDEL_PRIMITIVES_H
#  define SEIDEL_PRIMITIVES_H

#  include <iostream>
#  include <utility>

#  include "vec2.h"

#  ifdef SEIDEL_USE_SIGNED_INDEX
using AnyID = int;
#  else
using AnyID = unsigned int;
#  endif
using NodeID    = AnyID;
using VertexID  = AnyID;
using SegmentID = AnyID;
using RegionID  = AnyID;
using Depth     = short;

constexpr AnyID INVALID_INDEX = -1, INFINITY_INDEX = -2, ROOT_REGION_ID = 0, ROOT_NODE_ID = 0;
constexpr Depth INVALID_DEPTH = -1;

#  pragma warning(push)
#  pragma warning(disable : 26495)  // uninitialized on purpose
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

  std::string ToString() const;
};
#  pragma warning(pop)

struct Region
{
  Region();

  NodeID nodeID;
  VertexID high, low;
  SegmentID left, right;

  Depth depth;

  RegionID highNeighbors[2], lowNeighbors[2];

  std::string ToString() const;
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

  std::string ToString() const;
};

using Vertex   = Vec2;
using NodePair = std::pair<NodeID, NodeID>;

#  ifdef SHOW_TRACE
#    define TRACE(X) std::cout << X << std::endl;
#  else
#    define TRACE(X)
#  endif
#endif