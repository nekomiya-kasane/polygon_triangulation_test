#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>

#include "vec2.h"

using Index                          = unsigned int;
using Indexes                        = std::vector<Index>;
using NodeIndex                      = unsigned int;
using NodeIndexes                    = std::vector<NodeIndex>;
constexpr NodeIndex INFINITY_INDEX   = -1;
constexpr NodeIndex INVALID_INDEX    = -1;
constexpr NodeIndex DEGENRATED_INDEX = -2;
constexpr NodeIndex ROOT_NODE_ID     = 0;
using Depth                          = unsigned short;
constexpr Depth INVALID_DEPTH        = -1;
constexpr double INVALID_ANGLE_P     = 4;
constexpr double INVALID_ANGLE_N     = -4;

inline bool ValidID(Index id)
{
  return id != INVALID_INDEX;
}

struct Node
{
  enum Type : char
  {
    NONE,
    VERTEX,
    SEGMENT,
    REGION,  // leaf
    TODESTORY
  };
  Type type;
  Index nodeID = INVALID_INDEX;  // not used now

  NodeIndex left = INVALID_INDEX, right = INVALID_INDEX;
};

struct RegionNode : public Node
{
  RegionNode(Index ind) : Node{REGION} { nodeID = ind; }

  NodeIndex highVertexID = INVALID_INDEX, lowVertexID = INVALID_INDEX;

  NodeIndex hlID = INVALID_INDEX /* left high */, hrID = INVALID_INDEX /* right high */,
            llID = INVALID_INDEX /* left low */, lrID = INVALID_INDEX /* right low */;
};

struct VertexNode : public Node
{
  VertexNode(Index ind, const Vec2 &position) : Node{VERTEX, ind}, pos(position) {}
  Index pointID;
  Vec2 pos;
};

struct SegmentNode : public Node
{
  SegmentNode(Index id, VertexNode *low, VertexNode *high) : Node{SEGMENT, id} {}
  NodeIndex fromVertexID, toVertexID;
  bool downward;

  inline NodeIndex HigherVertexID() { return downward ? fromVertexID : toVertexID; }
  inline NodeIndex LowerVertexID() { return !downward ? fromVertexID : toVertexID; }
};

constexpr const size_t NODE_SIZE =
    std::max({sizeof(RegionNode), sizeof(SegmentNode), sizeof(VertexNode)});

template <Node::Type T>
struct NodeTrait
{};
template <>
struct NodeTrait<Node::Type::NONE>
{
  using type = Node;
};
template <>
struct NodeTrait<Node::Type::REGION>
{
  using type = RegionNode;
};
template <>
struct NodeTrait<Node::Type::SEGMENT>
{
  using type = SegmentNode;
};
template <>
struct NodeTrait<Node::Type::VERTEX>
{
  using type = VertexNode;
};
