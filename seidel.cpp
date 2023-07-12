#include "seidel.h"

#include <algorithm>
#include <cassert>
#include <random>

NodeIndex Triangulator::AddVectex(const Vec2 &pointLoc, Index pointID)
{
  auto &nodeID = _vertices[pointID];
  if (ValidID(nodeID)) /* already added */
    return nodeID;

  NodeIndex regionID = GetVertexRegion(pointID);
  SplitRegionByPoint(regionID, pointID);

  return nodeID;
}

SegmentNode *Triangulator::AddSegment(Index fromInd, Index toInd)
{
  Vec2 fromVert = _points[fromInd], toVert = _points[toInd];

  bool fromFirst = GenerateRandomBool(&fromInd);
  VertexNode *topNode, *bottomNode;
  if (fromFirst)
  {
    topNode    = AddVectex(fromVert, fromInd);
    bottomNode = AddVectex(toVert, toInd);
  }
  else
  {
    topNode    = AddVectex(toVert, toInd);
    bottomNode = AddVectex(fromVert, fromInd);
  }
  if (topNode->pos < bottomNode->pos)
    std::swap(topNode, bottomNode);

  RegionNode *curRegion = static_cast<RegionNode *>(topNode->right);
  SplitRegionByEdge(curRegion, topNode, bottomNode);
  UpdateFirstSplitRegion(curRegion);
  UpdateRegionsAbovePoints(curRegion);
  curRegion = FindRegionBelow(curRegion);
  while (curRegion != bottomNode->right)
  {
    SplitRegionByEdge(curRegion);
    UpdateMiddleSplitRegion(curRegion);
    UpdateRegionsAbovePoints();
    curRegion = FindRegionBelow(curRegion);
  }
  UpdateLastSplitRegion(curRegion);
}

void Triangulator::AddPolygon(const Vec2Set &vertices)
{
  size_t oldSize       = _points.size();
  size_t incrementSize = _compactPoints ? vertices.size() : vertices.size() - 1;

  _points.reserve(_points.size() + vertices.size());
  _points.insert(_points.end(), vertices.cbegin(), vertices.cbegin() + incrementSize);

  _permutation.reserve(_permutation.size() + incrementSize);
  _vertexCount += incrementSize;
  _segmentCount += incrementSize;

  for (size_t i = oldSize; i < oldSize + incrementSize; ++i)
    _permutation.push_back({i - 1, i});
  _permutation.push_back({oldSize + incrementSize - 1, oldSize});
}

NodeIndex Triangulator::SplitRegionByPoint(NodeIndex regionID, Index pointID)
{
  // create leaves
  NodeIndex higherRegionID = _alloc.DuplicateNode<Node::REGION>(regionID);
  NodeIndex lowerRegionID  = _alloc.DuplicateNode<Node::REGION>(regionID);

  RegionNode *higherRegion = _alloc.GetNode<Node::REGION>(higherRegionID),
             *lowerRegion  = _alloc.GetNode<Node::REGION>(lowerRegionID);

  // turn the parent region node into vertex node
  NodeIndex vertexID = regionID;
  VertexNode *vertex = _alloc.RecastNode<Node::VERTEX>(vertexID);
  vertex->pos        = _points[pointID];
  vertex->pointID    = pointID;
  vertex->left       = higherRegionID;
  vertex->right      = lowerRegionID;
  _vertices.push_back(vertexID);

  lowerRegion->highVertexID = vertexID;
  higherRegion->lowVertexID = vertexID;

  // todo: maintain neighbor
  higherRegion->llID = INVALID_INDEX;
  higherRegion->lrID = lowerRegionID;
  lowerRegion->hlID  = higherRegionID;
  lowerRegion->hrID  = INVALID_INDEX;
  // do I need to remember the first trapezoid?

  return lowerRegionID;
}

RegionNode *Triangulator::SplitRegionByEdge(RegionNode *region,
                                            const VertexNode *topVertex,
                                            const VertexNode *bottomVertex)
{
  RegionNode *leftRegion  = CreateNode<Node::REGION>(&region);
  RegionNode *rightRegion = CreateNode<Node::REGION>();

  SegmentNode *&&segment = CastNode<Node::SEGMENT>(region);
  segment->high          = topVertex;
  segment->low           = bottomVertex;
}

void Triangulator::BuildQueryTree()
{
  // presets
  UpdatePermutation();

  _vertices.resize(_vertexCount);
  _segments.resize(_segmentCount);

  // root
  _alloc.CreateNode<Node::REGION>();

  // leaves
  size_t baseInd = 0;
  while (true)
  {
    size_t end = std::min(baseInd + _phase, _permutation.size());
    for (size_t i = baseInd; i < end; ++i)
    {
      const Index fromInd = _permutation[i].first;
      const Index toInd   = _permutation[i].second;

      AddSegment(fromInd, toInd);
    }

    baseInd = end;
    if (baseInd >= _permutation.size())
      break;

    UpdateVertexPosition();
  }
}

RegionNode *Triangulator::Query(const Vec2 &point)
{
  return QueryFrom(_root, point);
}

RegionNode *Triangulator::QueryFrom(Node *node, const Vec2 &point)
{
  Node::Type type = node->type;
  while (type != Node::REGION)
  {
    if (type == Node::VERTEX)
    {
      VertexNode *&&vertex = static_cast<VertexNode *>(node);
      node                 = VertexHigher(point, vertex->pos) ? vertex->left : vertex->right;
    }
    else
    {
      assert(type == Node::SEGMENT);
      SegmentNode *&&segment = static_cast<SegmentNode *>(node);
      node = VertexLefter(point, segment->low->pos, segment->high->pos) ? segment->left
                                                                        : segment->right;
    }
  }
  return reinterpret_cast<RegionNode *&>(node);
}

NodeIndex &Triangulator::GetVertexRegion(NodeIndex vertexID)
{
  NodeIndex &regionID = _vertexTreePosition[vertexID];
  if (!ValidID(regionID))
    regionID = ROOT_NODE_ID;

  regionID = QueryFrom(regionID, _alloc.GetNode<Node::VERTEX>(vertexID)->pos);
  return regionID;
}

void Triangulator::UpdateVertexPosition()
{
  // remove
  // for (auto &&node : _nodeToDestroy)
  //  delete node;
  //_nodeToDestroy.clear();
}

void Triangulator::UpdatePermutation()
{
  std::random_device rd;
  std::mt19937 g(rd());

  std::shuffle(_permutation.begin(), _permutation.end(), g);
}

bool Triangulator::GenerateRandomBool(void *seed)
{
  return 0x100 & reinterpret_cast<size_t>(seed);
}

bool Triangulator::VertexHigher(const Vec2 &left, const Vec2 &right)
{
  if (left.y - right.y > _tol)
    return true;
  if (left.y - right.y < -_tol)
    return false;
  if (left.x - right.y <= 0)
    return true;
  return false;

  // todo: handle collision
}

const Triangles &Triangulator::Triangulate() {
  BuildQueryTree();
}