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

  ResolveIntersection(originalRegionID, segment);
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
  lowRegion.highNeighbors[1] = GET_REAL_ID(highRegion.nodeID);
  lowRegion.lowNeighbors[0]  = highRegion.lowNeighbors[0];
  lowRegion.lowNeighbors[1]  = highRegion.lowNeighbors[1];

  highRegion.lowNeighbors[0] = INVALID_INDEX;
  highRegion.lowNeighbors[1] = GET_REAL_ID(lowRegion.nodeID);

  // new NodeID for the original Region since it's now a leaf of the original node
  Node &newNodeForHighRegion = NewNode(Node::REGION);
  highRegion.nodeID          = newNodeForHighRegion.id;

  return {highRegion.nodeID, lowRegion.nodeID};
}

NodePair TrapezoidMap::SplitRegionBySegment(RegionID regionID,
                                            SegmentID segmentID,
                                            VertexID segmentHighVertexID,
                                            VertexID segmentLowVertexID,
                                            int type)
{
  // The segment won't intersect with the left/right segments of the region, we have ensure this
  // outside this function
  Region &highRegion = _regions[regionID];  // left
  Region &lowRegion  = NewRegion();         // right

  RegionID lowLeftRegionID  = highRegion.lowNeighbors[0],
           lowRightRegionID = highRegion.lowNeighbors[1];
  assert(Valid(lowLeftRegionID) && Valid(lowRightRegionID));
  bool oneRegionBelow = lowLeftRegionID == lowRightRegionID;

  // assign original NodeID to the overridden node
  Node &originalNode = _nodes[highRegion.nodeID];
  originalNode.type  = Node::SEGMENT;
  originalNode.value = segmentID;

  Node &newNodeForHighRegion = NewNode(Node::Type::REGION);
  highRegion.nodeID          = newNodeForHighRegion.id;  // renumber high region

  originalNode.left  = highRegion.nodeID;
  originalNode.right = lowRegion.nodeID;

  // sync info
  // bool toLowVertex = _segments[segmentID];
  switch (type)
  {
    case 0:
    {
      bool fromRightTop = _segments[segmentID].highVertex == _segments[highRegion.right].highVertex;
      bool toLow        = _segments[segmentID].lowVertex == highRegion.low;

      if (fromRightTop)
      {
        // segments
        highRegion.right = segmentID;
        lowRegion.left   = segmentID;

        if (toLow)
        {
          Region &lowLeftRegion  = _regions[highRegion.lowNeighbors[0]];
          Region &lowRightRegion = _regions[highRegion.lowNeighbors[1]];

          if (oneRegionBelow)
          {
            // degenerated 1: same edge added again - forbidden
            // ...
            // degenerated 2: diagonal
            if (_segments[highRegion.left].lowVertex == segmentLowVertexID)
            {
              lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] =
                  lowLeftRegionID;  // == lowRightRegionID
              highRegion.lowNeighbors[0] = highRegion.lowNeighbors[1] = INVALID_INDEX;

              lowLeftRegion.highNeighbors[0] = lowLeftRegion.highNeighbors[1] =
                  GET_REAL_ID(lowRegion.nodeID);

              // todo: update
            }
            // mid
            else
            {
              lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = lowRightRegionID;
              lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = lowLeftRegionID;
            }
          }
        }

        // resolve highRegion
      }
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

  // todo: handle intersection
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
  // todo: subRef
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

  int res = Higher(leftVertex, rightVertex);
  if (res > -1)
    return !!res;

  // same x, y: latter vertex always on the right, is this OK?
  assert(leftVertexID != rightVertexID);
  return leftVertexID < rightVertexID;
}

int TrapezoidMap::Higher(const Vertex &leftVertex, const Vertex &rightVertex) const
{
  if (leftVertex.y > rightVertex.y)
    return true;
  if (leftVertex.y < rightVertex.y)
    return false;

  // same y
  if (leftVertex.x < rightVertex.x)
    return true;
  if (leftVertex.x > rightVertex.x)
    return true;

  return -1;  // uncertain
}

bool TrapezoidMap::Higher(VertexID refVertexID,
                          VertexID highVertexID,
                          VertexID lowVertexID,
                          VertexID subRefVertexID) const
{
  const Vertex &refVertex = _vertices[refVertexID], &highVertex = _vertices[highVertexID],
               &lowVertex = _vertices[lowVertexID];
  int res                 = Higher(refVertex, highVertex, lowVertex,
                   Valid(subRefVertexID) ? &_vertices[subRefVertexID] : nullptr);
  if (res > -1)
    return !!res;

  return refVertexID < std::min(highVertexID, lowVertexID);
}

int TrapezoidMap::Higher(const Vertex &refVertex,
                         const Vertex &highVertex,
                         const Vertex &lowVertex,
                         const Vertex *const subRefVertexPtr) const
{
  auto highLow = highVertex - lowVertex;
  double cross = highLow ^ (refVertex - lowVertex);
  if (cross != 0.)
    return cross > 0;
  if (highLow.NormSq() == 0.)
    return Higher(refVertex, highVertex);

  if (!subRefVertexPtr)
    return false;

  const Vertex &subRefVertex = *subRefVertexPtr;
  auto refHighLow            = refVertex - subRefVertex;
  cross                      = refHighLow ^ highLow;
  if (cross != 0.)
    return cross > 0;
  if (refHighLow.NormSq() == 0.)
    return lowVertex.x > highVertex.x;

  // todo: still 0? Is this possible?
  return false;
}