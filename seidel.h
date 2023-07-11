#pragma once
#include <iostream>
#include <unordered_map>
#include <vector>

#include "vec2.h"

using Index                   = unsigned int;
using Indexes                 = std::vector<Index>;
constexpr Index INVALID_INDEX = (Index)-1;
using Depth                   = unsigned short;
constexpr Depth INVALID_DEPTH = (Depth)-1;

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
  Index id = INVALID_INDEX;  // not used now

  Node *left = nullptr, *right = nullptr;
};

struct RegionNode : public Node
{
  RegionNode(Index ind) : Node{REGION, ind} {}
  RegionNode(Index ind, const RegionNode &target) : RegionNode(target) { id = ind; }

  VertexNode *highVertex = nullptr /* null for infinite high */, *lowVertex = nullptr /* null for infinite low */;

  SegmentNode *leftSegment = nullptr, SegmentNode *rightSegment = nullptr;
  RegionNode *aboveRegion = nullptr, RegionNode *belowRegion = nullptr;
};

struct VertexNode : public Node
{
  VertexNode(Index ind, const Vec2 &position) : Node{VERTEX, ind}, pos(position) {}
  Vec2 pos;
  SegmentNode *segments[2] = {nullptr, nullptr};
};

struct SegmentNode : public Node
{
  SegmentNode(Index id, VertexNode *low, VertexNode *high) : Node{SEGMENT, id} {}
  VertexNode *low, *high;
};

constexpr const size_t NODE_SIZE = std::max({sizeof(RegionNode), sizeof(SegmentNode), sizeof(VertexNode)});

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

struct Triangle
{
  Index A, B, C;
};

using TriangleVector = std::vector<Triangle>;
using VertexVector   = std::vector<VertexNode *>;
using SegmentVector  = std::vector<SegmentNode *>;

class Triangulator
{
public:
  ~Triangulator();

  void AddPolygon(const Vec2Set &vertices);
  const TriangleVector &Triangulate();

protected:
  // procedure
  template <Node::Type T, class... Args>
  inline NodeTrait<T>::type *CreateNode(const Args &...args)
  {
    NodeTrait<T>::type *node = new (AllocNode()) NodeTrait<T>::type(_curNodeCount++, args);
    return node;
  }
  template <Node::Type T>
  inline NodeTrait<T>::type *CastNode(Node *node)
  {
    NodeTrait<T>::type *transnode = reinterpret_cast<NodeTrait<T>::type *>(node);
    transnode->type               = T;
    return transnode;
  }
  void SwapNode(Node *node1, Node *node2);
  void OverwriteNode(Node *dest, Node *src);
  void BuildQueryTree();

  RegionNode *Query(const Vec2 &point);
  RegionNode *QueryFrom(Node *node, const Vec2 &point);

  // tree
  VertexNode *AddVectex(const Vec2 &vertex, Index id);
  SegmentNode *AddSegment(Index fromID, Index toID);

  [[nodiscard]] RegionNode *SplitRegionByPoint(RegionNode *region, const Vec2 &point);
  [[nodiscard]] RegionNode *SplitRegionByEdge(RegionNode *region,
                                              const VertexNode *topVertex,
                                              const VertexNode *bottomVertex);

  RegionNode *&ResolveVertexRegionCache(Index id);
  void UpdateVertexPosition();

  RegionNode *FindRegionBelow(RegionNode *region) const;
  void UpdateFirstSplitRegion(RegionNode *region);
  void UpdateMiddleSplitRegion(RegionNode *region);
  void UpdateLastSplitRegion(RegionNode *region);
  void UpdateRegionsAbovePoints(RegionNode *region);

  TriangleVector TriangulateMonoMountain(const Indexes &vertices);
  void AssignDepth();

  void UpdatePermutation();
  static bool GenerateRandomBool(void *seed);
  bool VertexHigher(const Vec2 &left, const Vec2 &right);
  bool VertexLefter(const Vec2 &point, const Vec2 &lowEnd, const Vec2 &highEnd);

  // configuration
  bool _compactPoints     = false;
  bool _checkIntersection = false;
  unsigned short _phase   = 800;
  double _tol             = 1e-10;

  // input
  Vec2Set _points;
  std::vector<std::pair<Index, Index>> _permutation;

  // results
  TriangleVector _results;

  // variables
  VertexVector _vertices;
  SegmentVector _segments;
  Node *_root;

  size_t _regionCount = 0, _vertexCount = 0, _segmentCount = 0;

  std::vector<bool> _vertexFlag;
  std::unordered_map<Index, Node *> _vertexTreePosition;  // is

  // allocator
  inline Node *AllocNode()
  {
    char *res = _dataNext;
    reinterpret_cast<Node*>(res)->id = (_dataNext - _data) / NODE_SIZE;
    res += NODE_SIZE;
    return (Node *)res;
  }
  mutable size_t _curNodeCount = 0;
  // 
  char *_data = nullptr, *_dataTop = nullptr, mutable *_dataNext = nullptr;
};