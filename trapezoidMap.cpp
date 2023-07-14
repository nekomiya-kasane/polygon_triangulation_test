#pragma once

#include "trapezoidMap.h"

#include <cassert>

#define GET_REAL_ID(NODE_ID) _nodes[NODE_ID].value
#define GET_VERTEX(NODE_ID) _vertices[GET_REAL_ID(NODE_ID)]
#define GET_SEGMENT(NODE_ID) _segments[GET_REAL_ID(NODE_ID)]
#define GET_REGION(NODE_ID) _regions[GET_REAL_ID(NODE_ID)]

bool TrapezoidMap::AddVertex(VertexID vertexID)
{
  if (Valid(_vertexAdded[vertexID]))
    return true;

  RegionID vertexRegion = Query(_vertices[vertexID]);
  Node &originalNode    = _nodes[_regions[vertexRegion].nodeID];

  auto [highRegion, lowRegion] = SplitRegionByVertex(vertexRegion, vertexID);

  // origin cast to vertex type
  originalNode.type  = Node::VERTEX;
  originalNode.left  = highRegion;
  originalNode.right = lowRegion;
  originalNode.value = vertexID;

  return false;
}

bool TrapezoidMap::AddSegment(VertexID fromID, VertexID toID)
{
  // add from high to low
  SegmentID segmentID = _segmentMap[{fromID, toID}];  // complex
  Segment &segment    = _segments[segmentID];
  segment.downward    = Higher(fromID, toID);
  if (segment.downward)
  {
    segment.highVertex = fromID;
    segment.lowVertex  = toID;
  }
  else
  {
    segment.highVertex = toID;
    segment.lowVertex  = fromID;
  }

  bool highAdded = AddVertex(segment.highVertex), lowAdded = AddVertex(segment.lowVertex);

  RegionID originalRegionID = GetRegionNext(segment.highVertex, segment.lowVertex), nextRegionID;
  Region &originalRegion    = _regions[originalRegionID];

  auto [leftRegionID, rightRegionID] = SplitRegionBySegment(originalRegionID, segmentID, 0);

  while (
      Valid(nextRegionID = GetRegionNext(originalRegionID, segment.highVertex, segment.lowVertex)))
  {
    std::tie(leftRegionID, rightRegionID) = SplitRegionBySegment(originalRegionID, segmentID, 1);

    originalRegionID = nextRegionID;
  }
  std::tie(leftRegionID, rightRegionID) = SplitRegionBySegment(originalRegionID, segmentID, 2);
}

NodePair TrapezoidMap::SplitRegionByVertex(RegionID regionID, VertexID vertexID)
{
  Region &highRegion = _regions[regionID];  // left
  Region &lowRegion  = NewRegion();         // right

  // sync info
  lowRegion.high             = vertexID;
  lowRegion.low              = highRegion.low;
  lowRegion.left             = highRegion.left;
  lowRegion.right            = highRegion.right;
  lowRegion.highNeighbors[0] = GET_REAL_ID(highRegion.nodeID);
  lowRegion.lowNeighbors[0]  = highRegion.lowNeighbors[0];
  lowRegion.lowNeighbors[1]  = highRegion.lowNeighbors[1];

  highRegion.lowNeighbors[1] = GET_REAL_ID(lowRegion.nodeID);
  highRegion.lowNeighbors[0] = INVALID_INDEX;

  return {highRegion.nodeID, lowRegion.nodeID};
}

NodePair TrapezoidMap::SplitRegionBySegment(RegionID regionID,
                                            VertexID vertexID,
                                            SegmentID segmentID,
                                            int type)
{
  Region &highRegion = _regions[regionID];  // left
  Region &lowRegion  = NewRegion();         // right

  // sync info
  //bool toLowVertex = _segments[segmentID];
  switch (type)
  {
    case 0:
    {
      bool fromRightTop = _segments[segmentID].highVertex == _segments[highRegion.right].highVertex;
      if (fromRightTop)
      {}
    }
  }
}

RegionID TrapezoidMap::GetRegionNext(VertexID highVertex, VertexID refVertex)
{
  const auto &lowNeighbors = _lowNeighbors[highVertex];
  assert(lowNeighbors.size());
  if (lowNeighbors.size() == 1)
    return lowNeighbors.cbegin()->regionID;

  Vec2 diff    = _vertices[refVertex] - _vertices[highVertex];
  double angle = std::atan2(diff.y, diff.x);
  if (lowNeighbors.cbegin()->leftAngle < angle)
    return lowNeighbors.cbegin()->regionID;

  const VertexNeighborInfo *found = nullptr;
  for (const auto &neighbor : lowNeighbors)
  {
    if (angle > neighbor.leftAngle && angle < neighbor.rightAngle)
    {
      found = &neighbor;
      break;
    }
    Region &region = GET_REGION(neighbor.regionID);
    if (angle == neighbor.rightAngle && Higher(refVertex, region.low))
    {
      found = &neighbor;
      break;
    }
    if (angle == neighbor.leftAngle && Higher(region.low, refVertex))
    {
      found = &neighbor;
      break;
    }
  }
  assert(found);
  return GET_REAL_ID(found->regionID);
}

RegionID TrapezoidMap::GetRegionNext(RegionID curRegionID,
                                     VertexID highVertexID,
                                     VertexID lowVertexID)
{
  Region &region          = _regions[curRegionID];
  VertexID lowMidVertexID = region.low;
  assert(Valid(lowMidVertexID));
  if (lowMidVertexID == lowVertexID)
    return INVALID_INDEX;  // last

  // occasion 1: go left or right
  Vertex intersection;
  if (Intersected(GET_REAL_ID(region.left), highVertexID, lowVertexID, &intersection) &&
      InInterval(intersection.y, _vertices[region.low].y, _vertices[region.high].y))
  {
    return SplitSegmentByVertex(curRegionID, highVertexID, intersection, true);
  }
  if (Intersected(GET_REAL_ID(region.right), highVertexID, lowVertexID, &intersection) &&
      InInterval(intersection.y, _vertices[region.low].y, _vertices[region.high].y))
  {
    return SplitSegmentByVertex(curRegionID, highVertexID, intersection, false);
  }

  // occasion 2: go below
  //    - only 1 below
  if (!Valid(region.lowNeighbors[0]))
  {
    assert(Valid(region.lowNeighbors[1]));
    return region.lowNeighbors[1];
  }

  //    - 2 below
  if (Higher(lowMidVertexID, highVertexID, lowVertexID))
    return region.lowNeighbors[1];
  return region.lowNeighbors[0];
}

bool TrapezoidMap::VertexNeighborInfoComparator::operator()(const VertexNeighborInfo &left,
                                                            const VertexNeighborInfo &right) const
{
  if (left.leftAngle != left.rightAngle || right.leftAngle != right.rightAngle)
  {
    if (left.rightAngle <= right.leftAngle)
      return false;
    if (left.leftAngle >= right.rightAngle)
      return true;
  }
  if (left.rightAngle == right.leftAngle)
    return left.regionID < right.regionID;
  return left.rightAngle > right.leftAngle;
}

bool TrapezoidMap::Higher(VertexID leftVertexID, VertexID rightVertexID) const
{
  const Vertex &leftVertex = _vertices[leftVertexID], &rightVertex = _vertices[rightVertexID];

  if (leftVertex.y > rightVertex.y)
    return true;
  if (leftVertex.y < rightVertex.y)
    return false;

  // same y
  if (leftVertex.x < rightVertex.x)
    return true;
  if (leftVertex.x > rightVertex.x)
    return true;

  // same x, y: latter vertex always on the right, is this OK?
  assert(leftVertexID != rightVertexID);
  return leftVertexID < rightVertexID;
}

bool TrapezoidMap::Higher(VertexID refVertexID,
                          VertexID highVertexID,
                          VertexID lowVertexID,
                          VertexID subRefVertexID) const
{
  const Vertex &refVertex = _vertices[refVertexID], &highVertex = _vertices[highVertexID],
               &lowVertex = _vertices[lowVertexID];
  return Higher(refVertex, highVertex, lowVertex,
                Valid(subRefVertexID) ? &_vertices[subRefVertexID] : nullptr);
}

bool TrapezoidMap::Higher(const Vertex &refVertex,
                          const Vertex &highVertex,
                          const Vertex &lowVertex,
                          const Vertex *const subRefVertexPtr) const
{
  // todo: high = low?
  double cross = (highVertex - lowVertex) ^ (refVertex - lowVertex);
  if (cross != 0.)
    return cross > 0;

  if (!subRefVertexPtr)
    return false;

  const Vertex &subRefVertex = *subRefVertexPtr;
  cross                      = (refVertex - subRefVertex) ^ (highVertex - lowVertex);
  if (cross != 0.)
    return cross > 0;

  // todo: still 0?
  return false;
}