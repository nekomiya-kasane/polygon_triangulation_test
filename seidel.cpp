#include "seidel.h"

#include <algorithm>
#include <cassert>
#include <random>

#define GET_NODE(ID) _alloc.GetNode(ID)
#define GET_VERTEX(ID) _alloc.GetNode<Node::VERTEX>(ID)
#define GET_SEGMENT(ID) _alloc.GetNode<Node::SEGMENT>(ID)
#define GET_REGION(ID) _alloc.GetNode<Node::REGION>(ID)

const Triangles &Triangulator::Triangulate()
{
  RefinePhase();
  BuildQueryTree();
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

  for (size_t i = oldSize; i < oldSize + incrementSize - 1; ++i)
  {
    _permutation.push_back({i, i + 1});
  }
  _permutation.push_back({oldSize + incrementSize - 1, oldSize});
}

NodeIndex Triangulator::AddVectex(const Vec2 &pointLoc, Index pointID)
{
  auto &nodeID = _vertices[pointID];
  if (ValidID(nodeID)) /* already added */
    return nodeID;

  NodeIndex regionID = GetVertexRegion(pointID);
  SplitRegionByPoint(regionID, pointID);

  return nodeID;
}

constexpr int HIGH_REGION_LOW_REGION = 1, HIGH_SEGMENT_LOW_REGION = 2, LOW_LEFT_REGION = 3,
              LOW_RIGHT_REGION = 4;

SegmentNode *Triangulator::AddSegment(Index fromInd, Index toInd)
{
  // add vertices
  const Vec2 &fromVert = _points[fromInd], &toVert = _points[toInd];

  bool fromFirst = GenerateRandomBool(&fromInd), downward = fromFirst;
  NodeIndex highVertexID, lowVertexID;
  if (fromFirst)
  {
    highVertexID = AddVectex(fromVert, fromInd);
    lowVertexID  = AddVectex(toVert, toInd);
  }
  else
  {
    highVertexID = AddVectex(toVert, toInd);
    lowVertexID  = AddVectex(fromVert, fromInd);
  }

  VertexNode *highVertex = GET_VERTEX(highVertexID), *lowVertex = GET_VERTEX(lowVertexID);

  if (highVertex->pos < lowVertex->pos)
  {
    std::swap(highVertex, lowVertex);
    std::swap(highVertexID, lowVertexID);
    downward = !downward;
  }

  // process
  int occasion;
  NodeIndex curRegion = FindRegionBelow(highVertexID, lowVertex->pos, &occasion);
  SplitRegionByEdge(curRegion, highVertexID, lowVertexID, downward, occasion);
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

NodeIndex Triangulator::FindRegionBelow(NodeIndex vertexID, const Vec2 &ref, int *catagory)
{
  VertexNode *vertex = GET_VERTEX(vertexID);

  // occasion 1: below no edge
  // Node *possibleRegion = GET_NODE(vertex->right);  // not always a region
  // if (possibleRegion->type == Node::REGION)
  //{
  //  if (catagory)
  //    *catagory = GET_NODE(vertex->left)->type == Node::REGION ? HIGH_REGION_LOW_REGION
  //                                                             : HIGH_SEGMENT_LOW_REGION;
  //  return possibleRegion->nodeID;
  //}

  // occasion 2: below 1 edge
  // RegionNode *regionAbove      = GET_REGION(_aboveInfo[vertex->pointID].left),
  //           *regionBelowRight = GET_REGION(regionAbove->lrID);
  // SegmentNode *segmentBelow    = GET_SEGMENT(regionBelowRight->left);

  // VertexNode *higherVertex = GET_VERTEX(segmentBelow->HigherVertexID()),
  //            *lowerVertex  = GET_VERTEX(segmentBelow->LowerVertexID());

  // if (VertexLefter(ref, lowerVertex->pos, higherVertex->pos))
  //{
  //   if (catagory)
  //     *catagory = LOW_LEFT_REGION;
  //   return regionAbove->llID;
  // }
  // else
  //{
  //   if (catagory)
  //     *catagory = LOW_RIGHT_REGION;
  //   return regionAbove->lrID;
  // }

  auto &belowInfo = _belowInfo[vertex->pointID];

  // occasion 1: no edge below
  if (!ValidID(belowInfo.right))
  {
    if (catagory)
      *catagory = GET_NODE(vertex->left)->type == Node::REGION ? HIGH_REGION_LOW_REGION
                                                               : HIGH_SEGMENT_LOW_REGION;
    return belowInfo.left;
  }

  // occasion 2: 1 edge below
  RegionNode *regionBelowLeft = GET_REGION(belowInfo.left);
  SegmentNode *segmentBelow   = GET_SEGMENT(regionBelowLeft->right);
  VertexNode *higherVertex    = GET_VERTEX(segmentBelow->HigherVertexID()),
             *lowerVertex     = GET_VERTEX(segmentBelow->LowerVertexID());
  if (VertexLefter(ref, lowerVertex->pos, higherVertex->pos))
  {
    if (catagory)
      *catagory = LOW_LEFT_REGION;
    return belowInfo.left;
  }
  else
  {
    if (catagory)
      *catagory = LOW_LEFT_REGION;
    return belowInfo.right;
  }
}

NodeIndex Triangulator::SplitRegionByPoint(NodeIndex regionID, Index pointID)
{
  // create leaves
  NodeIndex higherRegionID = _alloc.DuplicateNode<Node::REGION>(regionID);
  NodeIndex lowerRegionID  = _alloc.DuplicateNode<Node::REGION>(regionID);

  RegionNode *higherRegion = GET_REGION(higherRegionID), *lowerRegion = GET_REGION(lowerRegionID);

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
  higherRegion->llID       = INVALID_INDEX;
  higherRegion->lrID       = lowerRegionID;
  lowerRegion->hlID        = higherRegionID;
  lowerRegion->hrID        = INVALID_INDEX;
  _belowInfo[pointID].left = lowerRegionID;
  // do I need to remember the first trapezoid?

  return lowerRegionID;
}

RegionNode *Triangulator::SplitRegionByEdge(NodeIndex regionID,
                                            NodeIndex highVertexID,
                                            NodeIndex lowVertexID,
                                            bool downward,
                                            int catagory)
{
  VertexNode *highVertex = GET_VERTEX(highVertexID);

  NodeIndex leftRegion  = _alloc.DuplicateNode<Node::REGION>(regionID);
  NodeIndex rightRegion = _alloc.DuplicateNode<Node::REGION>(regionID);

  SegmentNode *segment  = _alloc.RecastNode<Node::SEGMENT>(regionID);
  segment->fromVertexID = downward ? highVertexID : lowVertexID;
  segment->toVertexID   = downward ? lowVertexID : highVertexID;

  switch (catagory)
  {
    case HIGH_REGION_LOW_REGION:
    {
    }
  }
}

void Triangulator::BuildQueryTree()
{
  // presets
  UpdatePermutation();

  _vertices.resize(_vertexCount);
  _segments.resize(_segmentCount);
  _endIndices.resize(_vertexCount);
  _belowInfo.resize(_vertexCount);

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

NodeIndex Triangulator::Query(const Vec2 &point)
{
  return QueryFrom(ROOT_NODE_ID, point);
}

NodeIndex Triangulator::QueryFrom(NodeIndex nodeID, const Vec2 &point)
{
  NodeIndex nodeID = nodeID;
  Node *node       = _alloc.GetNode(nodeID);
  Node::Type type  = node->type;
  while (type != Node::REGION)
  {
    if (type == Node::VERTEX)
    {
      VertexNode *&&vertex = static_cast<VertexNode *>(node);
      nodeID               = VertexHigher(point, vertex->pos) ? vertex->left : vertex->right;
    }
    else
    {
      assert(type == Node::SEGMENT);
      SegmentNode *&&segment     = static_cast<SegmentNode *>(node);
      VertexNode *&&higherVertex = GET_VERTEX(segment->HigherVertexID());
      VertexNode *&&lowerVertex  = GET_VERTEX(segment->LowerVertexID());
      nodeID =
          VertexLefter(point, lowerVertex->pos, higherVertex->pos) ? segment->left : segment->right;
    }
  }
  return nodeID;
}

NodeIndex Triangulator::GetVertexRegion(NodeIndex vertexID)
{
  NodeIndex &regionID = _vertexTreePosition[vertexID];
  if (!ValidID(regionID))
    regionID = ROOT_NODE_ID;

  regionID = QueryFrom(regionID, GET_VERTEX(vertexID)->pos);
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

bool Triangulator::VertexLefter(const Vec2 &point, const Vec2 &low, const Vec2 &high)
{
  return ((point - low) ^ (high - low)) < 0;
}

void Triangulator::RefinePhase()
{
  if (!_phase)
  {
    unsigned short i = 0;
    float val        = _vertices.size();
    while (val >= 1.0f)
    {
      val = std::log(val);
      ++i;
    }
    _phase = i - 1;
  }
}