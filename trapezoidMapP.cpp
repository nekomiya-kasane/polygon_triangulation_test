#pragma once

#include "trapezoidMapP.h"

#include <cassert>

#define GET_REAL_ID(NODE_ID) _nodes[NODE_ID].value
#define GET_VERTEX(NODE_ID) _vertices[GET_REAL_ID(NODE_ID)]
#define GET_SEGMENT(NODE_ID) _segments[GET_REAL_ID(NODE_ID)]
#define GET_REGION(NODE_ID) _regions[GET_REAL_ID(NODE_ID)]

bool TrapezoidMapP::AddVertex(VertexID vertexID)
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

bool TrapezoidMapP::AddSegment(SegmentID segmentID)
{
  // add from high to low
  Segment &segment = _segments[segmentID];

  bool highAdded = AddVertex(segment.highVertex), lowAdded = AddVertex(segment.lowVertex);

  int type;
  RegionID originalRegionID = GetRegionNext(segment.highVertex, segment.lowVertex, &type);
  Region &originalRegion    = _regions[originalRegionID];

  ResolveIntersection(originalRegionID, segmentID, type != 2, type != -2);

  auto [leftRegionID, rightRegionID] = SplitRegionBySegment(originalRegionID, segmentID, 0);

  while (
      Valid(nextRegionID = GetRegionNext(originalRegionID, segment.highVertex, segment.lowVertex)))
  {
    std::tie(leftRegionID, rightRegionID) = SplitRegionBySegment(originalRegionID, segmentID, 1);

    originalRegionID = nextRegionID;
  }
  std::tie(leftRegionID, rightRegionID) = SplitRegionBySegment(originalRegionID, segmentID, 2);
}

RegionID TrapezoidMapP::Query(const Vertex &point)
{
  assert(_regions.Size());
  return QueryFrom(ROOT_NODE_ID, point);
}

RegionID TrapezoidMapP::QueryFrom(NodeID nodeID, const Vertex &point) {}

NodePair TrapezoidMapP::SplitRegionByVertex(RegionID regionID, VertexID vertexID)
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

NodePair TrapezoidMapP::SplitFirstRegionBySegment(RegionID regionID, SegmentID segmentID, int &type)
{
  // The segment won't intersect with the left/right segments of the region, we have ensured this
  // outside this function.
  // two vertices are already inserted, hence we need not to consider other occasions - the vertices
  // must be the high & low vertex of the trapezoid.

  Region &highRegion   = _regions[regionID];  // left
  Region &lowRegion    = NewRegion();         // right
  RegionID lowRegionID = GET_REAL_ID(lowRegion.nodeID);

  // update neighbors
  UpdateAbove(regionID, regionID, lowRegionID, segmentID, type);
  type = UpdateBelow(regionID, regionID, lowRegionID, segmentID);

  // assign original NodeID to the overridden node
  Node &originalNode = _nodes[highRegion.nodeID];
  originalNode.type  = Node::SEGMENT;
  originalNode.value = segmentID;

  Node &newNodeForHighRegion = NewNode(Node::Type::REGION);
  highRegion.nodeID          = newNodeForHighRegion.id;  // renumber high region

  originalNode.left  = highRegion.nodeID;
  originalNode.right = lowRegion.nodeID;
}

void TrapezoidMapP::UpdateAbove(Region &originalRegion,
                                Region &highRegion,
                                Region &lowRegion,
                                SegmentID segmentID,
                                int type)
{
  // update high region
  switch (type)
  {
    case -2:
    {
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      highRegion.highNeighbors[0] = highRegion.highNeighbors[1] = INVALID_INDEX;
      break;
    }
    case 0:
    {
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      highRegion.highNeighbors[1]                             = highRegion.highNeighbors[0];
      break;
    }
    case 2:
    {
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = INVALID_INDEX;
      break;
    }
    case -1:
    {
      lowRegion.highNeighbors[0]  = highRegion.highNeighbors[0];
      lowRegion.highNeighbors[1]  = highRegion.highNeighbors[1];
      highRegion.highNeighbors[1] = highRegion.highNeighbors[0];
      break;
    }
    case 1:
    {
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      break;
    }
  }

  // update left and right segment
  lowRegion.right = highRegion.right;
  lowRegion.left = highRegion.right = segmentID;
}

int TrapezoidMapP::UpdateBelow(RegionID originalRegionID,
                               RegionID highRegionID,
                               RegionID lowRegionID,
                               SegmentID segmentID)
{
  // merging already completed before calling this.
  Region &original = _regions[originalRegionID], &high = _regions[highRegionID],
         &low       = _regions[lowRegionID];
  Segment &segment  = _segments[segmentID];
  VertexID leftLow  = _segments[original.left].lowVertex,
           rightLow = _segments[original.right].lowVertex;

  if (original.low == segment.lowVertex)
  {
    _nextRegion = _tmpRegionToMerge = INVALID_INDEX;
    _mergeType                      = 0;
    low.lowNeighbors[0] = low.lowNeighbors[1] = original.lowNeighbors[1];

    if (leftLow == segment.lowVertex)
    {
      high.lowNeighbors[0] = high.lowNeighbors[1] = INVALID_INDEX;
    }
    if (rightLow == segment.lowVertex)
    {
      low.lowNeighbors[0] = low.lowNeighbors[1] = INVALID_INDEX;
    }
    return 0;
  }
  else
  {
    int res = Higher(original.low, segment.highVertex, segment.lowVertex) ? 1 : -1;
    if (res == 1)  // to low right
    {
      _nextRegion       = original.lowNeighbors[1];
      _tmpRegionToMerge = lowRegionID;
      _mergeType        = 1;  // merge low subregion

      low.lowNeighbors[0] = low.lowNeighbors[1] = original.lowNeighbors[1];
    }
    else
    {
      _nextRegion       = original.lowNeighbors[0];
      _tmpRegionToMerge = highRegionID;
      _mergeType        = -1;  // merge high subregion

      low.lowNeighbors[0]  = high.lowNeighbors[0];
      low.lowNeighbors[1]  = high.lowNeighbors[1];
      high.lowNeighbors[1] = high.lowNeighbors[0];
    }
    return res;
  }
}

RegionID TrapezoidMapP::GetRegionNext(VertexID highVertex,
                                      VertexID refVertex /* lower */,
                                      int *type)
{
  RegionID region;  // won't change after
  {
    const auto &lowNeighbors = _lowNeighbors[highVertex];
    auto &&lowerNeighborSize = lowNeighbors.Size();
    assert(lowerNeighborSize != 0 && lowerNeighborSize != 3);

    // occasion 1: 1 below
    // \-------*-----------/
    //  \       this      /
    //   \------------*--/

    if (lowerNeighborSize == 1)
    {
      if (type)
        *type = 0;
      region = lowNeighbors.left;
    }

    // occasion 2: 2 below, from middle
    else
    {
      const Segment &midSegment = _segments[_regions[lowNeighbors.left].right];
      if (Higher(refVertex, highVertex, midSegment.lowVertex))
      {
        // from right top
        if (type)
          *type = -2;
        region = lowNeighbors.left;
      }
      else
      {
        // from left top
        if (type)
          *type = 2;
        region = lowNeighbors.right;
      }
    }
  }

  return region;
}

void TrapezoidMapP::ResolveIntersection(RegionID curRegionID,
                                        SegmentID segmentID,
                                        bool checkLeft,
                                        bool checkRight)
{
  Region &region        = _regions[curRegionID];
  Segment &newSegment   = _segments[segmentID];
  VertexID highVertexID = newSegment.highVertex, lowVertexID = newSegment.lowVertex;

  bool newSegmentDownward      = newSegment.downward;
  SegmentID newNewSubsegmentID = INVALID_INDEX;

  Vertex intersection;
  if (checkLeft)
  {
    Segment &leftSegment = _segments[region.left];
    bool leftDownward    = leftSegment.downward;

    VertexID &leftHighVertexID = leftSegment.highVertex, &leftLowVertexID = leftSegment.lowVertex;
    if (Intersected(leftHighVertexID, leftLowVertexID, highVertexID, lowVertexID, &intersection))
    {
      // split vertices
      VertexID leftNewVertexID      = AppendVertex(intersection),
               rightNewVertexID     = AppendVertex(intersection);
      SegmentID newLeftSubsegmentID = AppendSegment(leftDownward);
      newNewSubsegmentID            = AppendSegment(newSegmentDownward);
      Segment &newLeftSubsegment    = _segments[newLeftSubsegmentID],
              &newNewSubsegment     = _segments[newNewSubsegmentID];
      newNewSubsegment.lowVertex    = lowVertexID;
      newLeftSubsegment.highVertex  = rightNewVertexID;
      newNewSubsegment.highVertex =
          (leftDownward == newSegmentDownward) ? leftNewVertexID : rightNewVertexID;
      newLeftSubsegment.lowVertex = leftLowVertexID;

      // find left intersected region
      RegionID leftRegionID = _lowNeighbors[leftHighVertexID].left;
      Region *leftRegion    = &_regions[leftRegionID];
      while (true)
      {
        RegionID leftBelowRegionID = leftRegion->lowNeighbors[1];  // right most
        Region &leftBelowRegion    = _regions[leftBelowRegionID];

        if (Higher(_vertices[leftBelowRegion.low], intersection))
        {
          leftRegionID = leftBelowRegionID;
          leftRegion   = &leftBelowRegion;
        }
      }

      SplitRegionByVertex(leftRegionID, leftNewVertexID);
      SplitRegionByVertex(curRegionID, rightNewVertexID);

      // resolve left regions
      VertexID footVertexID = leftRegion->low;
      while (footVertexID != leftLowVertexID)
      {
        leftRegionID = leftRegion->lowNeighbors[1];
        leftRegion   = &_regions[leftRegionID];

        // modify right segment
        leftRegion->right = newLeftSubsegmentID;
      }

      // resolve right regions
      Region *rightRegion = &region;
      footVertexID        = rightRegion->low;
      while (footVertexID != leftLowVertexID)
      {
        leftRegionID = leftRegion->lowNeighbors[0];
        leftRegion   = &_regions[leftRegionID];

        // modify right segment
        leftRegion->left = newLeftSubsegmentID;
      }

      // resolve segments
      if (leftDownward && newSegmentDownward)
      {
        _endVertices[leftHighVertexID]  = leftNewVertexID;
        _endVertices[leftNewVertexID]   = leftLowVertexID;
        _endVertices[highVertexID]      = rightNewVertexID;
        _endVertices[rightNewVertexID]  = lowVertexID;
        _prevVertices[leftNewVertexID]  = leftHighVertexID;
        _prevVertices[rightNewVertexID] = highVertexID;
        _prevVertices[lowVertexID]      = rightNewVertexID;
        _prevVertices[leftLowVertexID]  = leftNewVertexID;

        newSegment.lowVertex = rightNewVertexID;
      }
      else if (!leftDownward && newSegmentDownward)
      {
        _endVertices[leftLowVertexID]   = leftNewVertexID;
        _endVertices[leftNewVertexID]   = leftHighVertexID;
        _endVertices[highVertexID]      = leftNewVertexID;
        _endVertices[rightNewVertexID]  = lowVertexID;
        _prevVertices[lowVertexID]      = rightNewVertexID;
        _prevVertices[rightNewVertexID] = leftLowVertexID;
        _prevVertices[leftHighVertexID] = leftNewVertexID;
        _prevVertices[leftNewVertexID]  = highVertexID;

        newSegment.lowVertex = leftNewVertexID;
      }
      else if (leftDownward && !newSegmentDownward)
      {
        _endVertices[leftHighVertexID]  = leftNewVertexID;
        _endVertices[lowVertexID]       = rightNewVertexID;
        _endVertices[rightNewVertexID]  = leftLowVertexID;
        _endVertices[leftNewVertexID]   = highVertexID;
        _prevVertices[leftNewVertexID]  = leftHighVertexID;
        _prevVertices[rightNewVertexID] = lowVertexID;
        _prevVertices[leftLowVertexID]  = rightNewVertexID;
        _prevVertices[highVertexID]     = leftNewVertexID;

        newSegment.lowVertex = leftNewVertexID;
      }
      else
      {
        _endVertices[leftNewVertexID]   = leftHighVertexID;
        _endVertices[rightNewVertexID]  = highVertexID;
        _endVertices[lowVertexID]       = rightNewVertexID;
        _endVertices[leftLowVertexID]   = leftNewVertexID;
        _prevVertices[leftHighVertexID] = leftNewVertexID;
        _prevVertices[leftNewVertexID]  = leftLowVertexID;
        _prevVertices[highVertexID]     = rightNewVertexID;
        _prevVertices[rightNewVertexID] = lowVertexID;

        newSegment.lowVertex = rightNewVertexID;
      }
    }
  }

  if (checkRight)
  {
    Segment &rightSegment = _segments[region.right];
    bool rightDownward    = rightSegment.downward;

    VertexID &rightHighVertexID = rightSegment.highVertex,
             &rightLowVertexID  = rightSegment.lowVertex;
    if (Intersected(rightHighVertexID, rightLowVertexID, highVertexID, lowVertexID, &intersection))
    {
      // split vertices
      VertexID leftNewVertexID       = AppendVertex(intersection),
               rightNewVertexID      = AppendVertex(intersection);
      SegmentID newRightSubsegmentID = AppendSegment(rightDownward);
      newNewSubsegmentID             = AppendSegment(newSegmentDownward);
      Segment &newRightSubsegment    = _segments[newRightSubsegmentID],
              &newNewSubsegment      = _segments[newNewSubsegmentID];
      newRightSubsegment.highVertex =
          (rightDownward == newSegmentDownward) ? rightNewVertexID : leftNewVertexID;
      newNewSubsegment.highVertex  = rightNewVertexID;
      newNewSubsegment.lowVertex   = lowVertexID;
      newRightSubsegment.lowVertex = rightLowVertexID;

      // find right intersected region
      RegionID rightRegionID = _lowNeighbors[rightHighVertexID].left;
      Region *rightRegion    = &_regions[rightRegionID];
      while (true)
      {
        RegionID rightBelowRegionID = rightRegion->lowNeighbors[0];  // left most
        Region &rightBelowRegion    = _regions[rightBelowRegionID];

        if (Higher(_vertices[rightBelowRegion.low], intersection))
        {
          rightRegionID = rightBelowRegionID;
          rightRegion   = &rightBelowRegion;
        }
      }

      SplitRegionByVertex(rightRegionID, leftNewVertexID);
      SplitRegionByVertex(curRegionID, rightNewVertexID);

      // resolve right regions
      VertexID footVertexID = rightRegion->low;
      while (footVertexID != rightLowVertexID)
      {
        rightRegionID = rightRegion->lowNeighbors[0];
        rightRegion   = &_regions[rightRegionID];

        // modify right segment
        rightRegion->left = newRightSubsegmentID;
      }

      // resolve left regions
      Region *rightRegion = &region;
      footVertexID        = rightRegion->low;
      while (footVertexID != rightLowVertexID)
      {
        rightRegionID = rightRegion->lowNeighbors[1];
        rightRegion   = &_regions[rightRegionID];

        // modify right segment
        rightRegion->right = newRightSubsegmentID;
      }

      // resolve segments
      if (rightDownward && newSegmentDownward)
      {
        _endVertices[rightHighVertexID] = rightNewVertexID;
        _endVertices[leftNewVertexID]   = rightLowVertexID;
        _endVertices[highVertexID]      = leftNewVertexID;
        _endVertices[rightNewVertexID]  = lowVertexID;
        _prevVertices[leftNewVertexID]  = highVertexID;
        _prevVertices[rightNewVertexID] = rightHighVertexID;
        _prevVertices[lowVertexID]      = rightNewVertexID;
        _prevVertices[rightLowVertexID] = leftNewVertexID;
      }
      else if (!rightDownward && newSegmentDownward)
      {
        _endVertices[rightLowVertexID]   = rightNewVertexID;
        _endVertices[leftNewVertexID]    = rightHighVertexID;
        _endVertices[highVertexID]       = leftNewVertexID;
        _endVertices[rightNewVertexID]   = lowVertexID;
        _prevVertices[lowVertexID]       = rightNewVertexID;
        _prevVertices[rightNewVertexID]  = rightLowVertexID;
        _prevVertices[rightHighVertexID] = leftNewVertexID;
        _prevVertices[leftNewVertexID]   = highVertexID;
      }
      else if (rightDownward && !newSegmentDownward)
      {
        _endVertices[rightHighVertexID] = leftNewVertexID;
        _endVertices[lowVertexID]       = rightNewVertexID;
        _endVertices[rightNewVertexID]  = rightLowVertexID;
        _endVertices[leftNewVertexID]   = highVertexID;
        _prevVertices[leftNewVertexID]  = rightHighVertexID;
        _prevVertices[rightNewVertexID] = lowVertexID;
        _prevVertices[rightLowVertexID] = rightNewVertexID;
        _prevVertices[highVertexID]     = leftNewVertexID;
      }
      else
      {
        _endVertices[leftNewVertexID]    = highVertexID;
        _endVertices[rightNewVertexID]   = rightHighVertexID;
        _endVertices[lowVertexID]        = rightNewVertexID;
        _endVertices[rightNewVertexID]   = leftNewVertexID;
        _prevVertices[rightHighVertexID] = rightNewVertexID;
        _prevVertices[leftNewVertexID]   = rightLowVertexID;
        _prevVertices[highVertexID]      = leftNewVertexID;
        _prevVertices[rightNewVertexID]  = lowVertexID;
      }

      newSegment.lowVertex = leftNewVertexID;
    }
  }
}

RegionID TrapezoidMapP::GetRegionNext(RegionID curRegionID,
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

NodeID TrapezoidMapP::SplitSegmentByVertex(RegionID lastRegionID,
                                           VertexID highVertexID,
                                           const Vertex &intersectionVertex,
                                           bool leftIntersected)
{}

bool TrapezoidMapP::Higher(VertexID leftVertexID, VertexID rightVertexID) const
{
  const Vertex &leftVertex = _vertices[leftVertexID], &rightVertex = _vertices[rightVertexID];

  int res = Higher(leftVertex, rightVertex);
  if (res > -1)
    return !!res;

  // same x, y: latter vertex always on the right, is this OK?
  assert(leftVertexID != rightVertexID);
  return leftVertexID < rightVertexID;
}

int TrapezoidMapP::Higher(const Vertex &leftVertex, const Vertex &rightVertex) const
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

bool TrapezoidMapP::Higher(VertexID refVertexID, VertexID highVertexID, VertexID lowVertexID) const
{
  const Vertex &refVertex = _vertices[refVertexID], &highVertex = _vertices[highVertexID],
               &lowVertex = _vertices[lowVertexID];
  int res                 = Higher(refVertex, highVertex, lowVertex);
  if (res > -1)
    return !!res;

  return refVertexID < std::min(highVertexID, lowVertexID);
}

int TrapezoidMapP::Higher(const Vertex &refVertex,
                          const Vertex &highVertex,
                          const Vertex &lowVertex) const
{
  auto highLow = highVertex - lowVertex;
  double cross = highLow ^ (refVertex - lowVertex);
  if (cross != 0.)
    return cross > 0;

  return -1;
}