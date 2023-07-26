#pragma once

#include "trapezoidMapP.h"

#include <cassert>
#include <queue>
#include <random>

#define GET_REAL_ID(NODE_ID) _nodes[NODE_ID].value
#define GET_VERTEX(NODE_ID)  _vertices[GET_REAL_ID(NODE_ID)]
#define GET_SEGMENT(NODE_ID) _segments[GET_REAL_ID(NODE_ID)]
#define GET_REGION(NODE_ID)  _regions[GET_REAL_ID(NODE_ID)]

void TrapezoidMapP::AddPolygon(const Vec2Set &points, bool compactPoints)
{
  unsigned int oldSize       = _vertices.Size(),
               incrementSize = static_cast<unsigned int>(compactPoints ? points.size() : points.size() - 1);

  _vertices.Reserve(_vertices.Size() + incrementSize);
  _segments.Reserve(_endVertices.Size() + incrementSize);
  _endVertices.ResizeRaw(_endVertices.Size() + incrementSize);
  _prevVertices.ResizeRaw(_prevVertices.Size() + incrementSize);

  _vertices.Pushback(points.data(), incrementSize);

  auto appendSegment = [this](VertexID from, VertexID to) {
    bool downward = Higher(from, to);

    SegmentID segmentID = this->AppendSegment(downward);
    Segment &segment    = _segments[segmentID];

    segment.highVertex = downward ? from : to;
    segment.lowVertex  = downward ? to : from;

    _endVertices[from] = to;
    _prevVertices[to]  = from;
  };

  for (VertexID i = oldSize; i < static_cast<VertexID>(oldSize + incrementSize - 1); ++i)
    appendSegment(i, i + 1);
  appendSegment(oldSize + incrementSize - 1, oldSize);
}

void TrapezoidMapP::Build()
{
  // generate permutation
  _permutation.reserve(_segments.Size());
  for (SegmentID i = 0, n = _segments.Size(); i < n; ++i)
    _permutation.push_back(i);

  std::random_device rd;
  if (config.useGivenSeed)
  {
    std::mt19937 g(config.seed);
    std::shuffle(_permutation.begin(), _permutation.end(), g);
  }
  else
  {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(_permutation.begin(), _permutation.end(), g);
  }

  _vertexRegions.resize(_vertices.Size(), ROOT_NODE_ID);
  _nodes.Reserve(_vertices.Size() * 3 + _segments.Size() * 3 + 6);
  _regions.Reserve(_vertices.Size() * 2 + _segments.Size() * 2 + 6);
  _lowNeighbors.resize(_vertices.Size());

  // root
  Region &rootRegion = NewRegion();
  rootRegion.high = rootRegion.low = INFINITY_INDEX;
  rootRegion.left = rootRegion.right = INFINITY_INDEX;
  rootRegion.highNeighbors[0] = rootRegion.highNeighbors[1] = INFINITY_INDEX;
  rootRegion.lowNeighbors[0] = rootRegion.lowNeighbors[1] = INFINITY_INDEX;

  // refine phase
  if (!config.phase)
  {
    unsigned int i = 0;
    double n       = _vertices.Size();
    while (n >= 1.0f)
    {
      n = std::log(n);
      ++i;
    }
    config.phase = i < 10 ? 10 : i - 1;
  }

  // leaves
  size_t baseInd = 0;
  while (true)
  {
    // one phase
    size_t end = std::min(baseInd + config.phase, _permutation.size());
    for (size_t i = baseInd; i < end; ++i)
      AddSegment(_permutation[i]);

    baseInd = end;
    if (baseInd >= _permutation.size())
      break;

    // update vertex
    for (size_t i = 0; i < _vertexRegions.size(); ++i)
    {
      NodeID &regionNodeID = _vertexRegions[i];
      if (!Valid(regionNodeID))
        continue;  // already added

      regionNodeID = _regions[QueryFrom(regionNodeID, (VertexID)i)].nodeID;
    }
  }

  // assign depth
  AssignDepth();
}

bool TrapezoidMapP::AddVertex(VertexID vertexID, NodeID startNodeID)
{
  if (!Valid(_vertexRegions[vertexID]))
    return true;

  RegionID vertexRegion = QueryFrom(startNodeID, vertexID);
  Node &originalNode    = _nodes[_regions[vertexRegion].nodeID];

  auto [highRegion, lowRegion] = SplitRegionByVertex(vertexRegion, vertexID);

  // origin cast to vertex type
  originalNode.type  = Node::VERTEX;
  originalNode.left  = highRegion;
  originalNode.right = lowRegion;
  originalNode.value = vertexID;

  _vertexRegions[vertexID] = INVALID_INDEX;  // invalid for already added vertex

  return false;
}

bool TrapezoidMapP::AddSegment(SegmentID segmentID)
{
  // add from high to low
  Segment &segment = _segments[segmentID];

  bool highAdded = AddVertex(segment.highVertex), lowAdded = AddVertex(segment.lowVertex);

  SegmentID splittedSegment = INVALID_INDEX;

  while (Valid(segmentID))
  {
    int type;
    RegionID originalRegionID = GetFirstIntersectedRegion(segment.highVertex, segment.lowVertex, &type);
    Region &originalRegion    = _regions[originalRegionID];

    if (config.checkIntersection)
      splittedSegment = ResolveIntersection(originalRegionID, segmentID, type != 2, type != -2);

    SplitRegionBySegment(originalRegionID, segmentID, type);

    while (Valid(_nextRegion))
    {
      originalRegionID = _nextRegion;
      if (config.checkIntersection)
      {
        splittedSegment = ResolveIntersection(originalRegionID, segmentID, type != 2, type != -2);
        assert(!Valid(splittedSegment));
      }
      SplitRegionBySegment(originalRegionID, segmentID, type);
    }

    segmentID = splittedSegment;
  }

  return true;
}

RegionID TrapezoidMapP::Query(VertexID vertexIDtoQuery)
{
  assert(_regions.Size());
  return QueryFrom(ROOT_NODE_ID, vertexIDtoQuery);
}

RegionID TrapezoidMapP::QueryFrom(NodeID nodeID, VertexID vertexIDtoQuery)
{
  Node *node      = &_nodes[nodeID];
  Node::Type type = node->type;
  while (type != Node::REGION)
  {
    if (type == Node::VERTEX)
    {
      VertexID vertexID = node->value;
      node              = Higher(vertexIDtoQuery, vertexID) ? &_nodes[node->left] : &_nodes[node->right];
    }
    else
    {
      assert(type == Node::SEGMENT);
      const Segment &segment = _segments[node->value];
      node = Higher(vertexIDtoQuery, segment.highVertex, segment.lowVertex) ? &_nodes[node->left]
                                                                            : &_nodes[node->right];
    }
    type = node->type;
  }
  return node->value;
}

VertexID TrapezoidMapP::AppendVertex(const Vertex &vertex)
{
  assert(_vertices.Size() == _endVertices.Size());
  assert(_vertices.Size() == _prevVertices.Size());
  assert(_vertices.Size() == _vertexRegions.size());

  VertexID id = _vertices.Pushback(vertex);

  _endVertices.Pushback(INVALID_INDEX);
  _prevVertices.Pushback(INVALID_INDEX);
  _vertexRegions.push_back(ROOT_NODE_ID);
  _lowNeighbors.emplace_back();

  return id;
}

SegmentID TrapezoidMapP::AppendSegment(bool dir)
{
  SegmentID id = _segments.Pushback(Segment{INVALID_INDEX, INVALID_INDEX, dir});
  return id;
}

NodePair TrapezoidMapP::SplitRegionByVertex(RegionID regionID, VertexID vertexID)
{
  Region &highRegion   = _regions[regionID];  // left
  Region &lowRegion    = NewRegion();         // right
  RegionID lowRegionID = _regions.GetIndex(&lowRegion);

  // update the newly created lowRegion
  lowRegion.high             = vertexID;
  lowRegion.low              = highRegion.low;
  lowRegion.left             = highRegion.left;
  lowRegion.right            = highRegion.right;
  lowRegion.highNeighbors[0] = GET_REAL_ID(highRegion.nodeID);
  lowRegion.highNeighbors[1] = GET_REAL_ID(highRegion.nodeID);
  lowRegion.lowNeighbors[0]  = highRegion.lowNeighbors[0];
  lowRegion.lowNeighbors[1]  = highRegion.lowNeighbors[1];

  // update the high neighbor of the lower neighbor of this region
  for (int i = 0; i < 2; ++i)
  {
    RegionID lowNeighborID = highRegion.lowNeighbors[i];
    if (Valid(lowNeighborID))
    {
      Region &lowNeighbor = _regions[lowNeighborID];
      if (lowNeighbor.highNeighbors[0] == regionID)
        lowNeighbor.highNeighbors[0] = lowRegionID;
      if (lowNeighbor.highNeighbors[1] == regionID)
        lowNeighbor.highNeighbors[1] = lowRegionID;
    }
  }

  // update highRegion
  highRegion.low             = vertexID;
  highRegion.lowNeighbors[0] = GET_REAL_ID(lowRegion.nodeID);
  highRegion.lowNeighbors[1] = GET_REAL_ID(lowRegion.nodeID);

  // new NodeID for the original Region since it's now a leaf of the original node
  Node &newNodeForHighRegion = NewNode(Node::REGION, regionID);
  highRegion.nodeID          = newNodeForHighRegion.id;

  // maintain neighbors
  _lowNeighbors[vertexID].left = lowRegionID;

  return {highRegion.nodeID, lowRegion.nodeID};
}

void TrapezoidMapP::SplitRegionBySegment(RegionID regionID, SegmentID segmentID, int &type)
{
  // The segment won't intersect with the left/right segments of the region, we have ensured this
  // outside this function.
  // two vertices are already inserted, hence we need not to consider other occasions - the vertices
  // must be the high & low vertex of the trapezoid.

  Region &highRegion = _regions[_mergeType == -1 ? _tmpRegionToMerge : regionID];
  Region &lowRegion  = _mergeType == 1 ? _regions[_tmpRegionToMerge] : NewRegion();

  RegionID lowRegionID = GET_REAL_ID(lowRegion.nodeID);

  // update neighbors
  UpdateAbove(highRegion, highRegion, lowRegion, segmentID, type);
  type = UpdateBelow(regionID, regionID, lowRegionID, segmentID);

  // assign original NodeID to the overridden node
  Node &originalNode = _nodes[highRegion.nodeID];
  originalNode.type  = Node::SEGMENT;
  originalNode.value = segmentID;

  Node &newNodeForHighRegion = NewNode(Node::Type::REGION, regionID);
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
  auto &highVertLowNei = _lowNeighbors[originalRegion.high];
  RegionID lowRegionID = _regions.GetIndex(&lowRegion);

  switch (type)
  {
    //   --*----------------------|
    //     |\                     |
    //     | \       Low          |
    //     |H \ <--- added        |
    case -2:
    {
      // update this region's high neighbor's low neighbors
      Region &highNeiRegion         = _regions[originalRegion.highNeighbors[1]];
      highNeiRegion.lowNeighbors[1] = lowRegionID;

      // update this region's neighbors
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      highRegion.highNeighbors[0] = highRegion.highNeighbors[1] = INVALID_INDEX;
      lowRegion.high                                            = highRegion.high;

      // update _lowNeighbors
      assert(Valid(highVertLowNei.right));
      highVertLowNei.mid   = highVertLowNei.right;
      highVertLowNei.right = lowRegionID;
      break;
    }

    case 0:
    {
      // update this region's high neighbor's low neighbors
      RegionID highNeiRegionHighID = originalRegion.highNeighbors[0],
               highNeiRegionLowID  = originalRegion.highNeighbors[1];
      Region &highNeiRegionHigh    = _regions[highNeiRegionHighID];

      if (highNeiRegionHighID != highNeiRegionLowID)
      {
        //   |        |               |
        //   |--------*---------------|
        //   |         \     Low      |
        //   | High     \             |
        //   |           \ <--- added |

        Region &highNeiRegionLow         = _regions[highNeiRegionLowID];
        highNeiRegionLow.lowNeighbors[0] = highNeiRegionLow.lowNeighbors[1] = lowRegionID;
      }
      else
      {
        //   |                        |
        //   |--------*---------------|
        //   |         \     Low      |
        //   | High     \             |
        //   |           \ <--- added |

        highNeiRegionHigh.lowNeighbors[1] = lowRegionID;
      }

      // update this region's neighbors
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = originalRegion.highNeighbors[1];
      highRegion.highNeighbors[1]                             = originalRegion.highNeighbors[0];
      lowRegion.high                                          = highRegion.high;

      // update _lowNeighbors
      assert(!Valid(highVertLowNei.mid) && (highVertLowNei.left == _regions.GetIndex(&highRegion)));
      highVertLowNei.right = lowRegionID;
      break;
    }

    //   |------------------------*--
    //   |                       /|
    //   |      High            / |
    //   |          added ---> / L|
    case 2:
    {
      // update this region's neighbors
      // already INVALID_INDEX by default ->
      //      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = INVALID_INDEX;
      assert(!Valid(lowRegion.highNeighbors[0]) && !Valid(lowRegion.highNeighbors[1]));
      lowRegion.high = highRegion.high;

      // update _lowNeighbors
      assert(highVertLowNei.left == _regions.GetIndex(&highRegion));
      highVertLowNei.mid = lowRegionID;
      break;
    }

    //   |-.-.-.-.\---*------------|
    //   |  High   \               |
    //   | (Merge)  \     Low      |
    //   |           \ <--- added  |
    case -1:
    {
      // update this region's high neighbor's low neighbors
      Region &highNeiRegion         = _regions[originalRegion.highNeighbors[1]];
      highNeiRegion.lowNeighbors[0] = highNeiRegion.lowNeighbors[1] = lowRegionID;

      // update this region's neighbors
      lowRegion.highNeighbors[0] = highRegion.highNeighbors[0];
      lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      // wrong!! -> highRegion.highNeighbors[1] = highRegion.highNeighbors[0];

      // update _lowNeighbors
      highVertLowNei.left = highVertLowNei.right = lowRegionID;
      break;
    }

    //   |-----------*-----/-.-.-.-|
    //   |    High        /  Low   |
    //   |               / (Merge) |
    //   |   added ---> /          |
    case 1:
    {
      // wrong!! -> lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      // nothing to do actually.
      break;
    }
  }

  // update left and right segment
  lowRegion.right = highRegion.right;
  lowRegion.left = highRegion.right = segmentID;

  // We modify high neighbors and segments in this method, but don't modify low neighbors and vertices
  // Whence its safe to query high/low vertices and low neighbors on highRegion in `UpdateBelow`.
}

int TrapezoidMapP::UpdateBelow(RegionID originalRegionID,
                               RegionID highRegionID,
                               RegionID lowRegionID,
                               SegmentID segmentID)
{
  assert(Valid(segmentID) && Valid(originalRegionID) && Valid(highRegionID) && Valid(lowRegionID));

  // merging already completed before calling this.
  Region &original = _regions[originalRegionID], &high = _regions[highRegionID], &low = _regions[lowRegionID];
  Segment &segment = _segments[segmentID];
  // high left is the left of original region, low right is the right of original region
  VertexID leftLowVertex  = !Infinite(high.left) ? _segments[high.left].lowVertex : INVALID_INDEX,
           rightLowVertex = !Infinite(low.right) ? _segments[low.right].lowVertex : INVALID_INDEX;

  // the last region
  if (original.low == segment.lowVertex)
  {
    if (leftLowVertex == segment.lowVertex)
    {
      //  \..../............|
      //   \../.<---.added..|
      //    \/..............|
      //-----*--------------|
      //                    |

      // update this region's low neighbor's high neighbors
      Region &lowNeiRegion          = _regions[original.lowNeighbors[1]];
      lowNeiRegion.highNeighbors[1] = lowRegionID;

      // update this region's neighbors
      low.lowNeighbors[0] = low.lowNeighbors[1] = original.lowNeighbors[1];
      high.lowNeighbors[0] = high.lowNeighbors[1] = INVALID_INDEX;
      high.low = low.low = leftLowVertex;
    }
    else if (rightLowVertex == segment.lowVertex)
    {
      //  |...........\..../
      //  |.added.--->.\../
      //  |.............\/
      //  |-------------*---
      //  |

      low.lowNeighbors[0] = low.lowNeighbors[1] = INVALID_INDEX;
      high.low = low.low = rightLowVertex;
    }
    else
    {
      // low neighbors of the low vertex of the original region
      const auto &oriLowVertLowNei = _lowNeighbors[original.low];
      Region &lowNeiRegion         = _regions[original.lowNeighbors[1]];

      high.lowNeighbors[0] = high.lowNeighbors[1] = oriLowVertLowNei.left;
      high.low = low.low = original.low;

      if (Valid(oriLowVertLowNei.right))
      {
        //  |...........\.....|
        //  |.added.--->.\....|
        //  |.............\...|
        //  |-------------*---|
        //  |            /    |

        // update this region's neighbors
        assert(oriLowVertLowNei.Size() == 2);
        low.lowNeighbors[0] = low.lowNeighbors[1] = oriLowVertLowNei.right;

        // update this region's low neighbor's high neighbors
        lowNeiRegion.highNeighbors[0] = lowNeiRegion.highNeighbors[1] = lowRegionID;
      }
      else
      {
        //  |...........\.....|
        //  |.added.--->.\....|
        //  |.............\...|
        //  |-------------*---|
        //  |                 |

        // update this region's neighbors
        low.lowNeighbors[0] = low.lowNeighbors[1] = oriLowVertLowNei.left;

        // update this region's low neighbor's high neighbors
        lowNeiRegion.highNeighbors[1] = lowRegionID;
      }
    }

    // no next region anymore
    _nextRegion = _tmpRegionToMerge = INVALID_INDEX;
    _mergeType                      = 0;
    return 0;
  }
  // not the last region
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

RegionID TrapezoidMapP::GetFirstIntersectedRegion(VertexID highVertex,
                                                  VertexID refVertex /* lower */,
                                                  int *type) const
{
  RegionID region;  // won't change after
  {
    const auto &lowNeighbors = _lowNeighbors[highVertex];
    auto &&lowerNeighborSize = lowNeighbors.Size();
    assert(lowerNeighborSize == 1 || lowerNeighborSize == 2);

    // occasion 1: 1 below
    //   |                        |      |        |               |
    //   |--------*---------------|      |--------*---------------|
    //   |.........\..............|  or  |.........\..............|
    //   |..........\.<---.added..|      |..........\.<---.added..|
    //   |.this.region............|      |.this.region............|
    //   |........................|      |........................|

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
        //   |------------------------*--
        //   |                       /|↖ highVertex
        //   | this.region          / |
        //   |          added ---> /  |
        //   |                    .   |
        //   |                   .    |
        //   |                  *     |
        //   |       refVertex ↗      |↙ midSegment.lowVertex
        //   |                     ---*---

        if (type)
          *type = 2;
        region = lowNeighbors.left;
      }
      else
      {
        //    ↙ highVertex
        // --*------------------------|
        //   |\                       |
        //   | \   this.region        |
        //   |  \ <--- added          |
        //   |   .                    |
        //   |    .                   |
        //   |     *                  |
        //   |      ↖ refVertex       |
        // --*--
        //    ↖ midSegment.lowVertex

        if (type)
          *type = -2;
        region = lowNeighbors.right;
      }
    }
  }

  return region;
}

SegmentID TrapezoidMapP::ResolveIntersection(RegionID curRegionID,
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
      VertexID leftNewVertexID = AppendVertex(intersection), rightNewVertexID = AppendVertex(intersection);
      SegmentID newLeftSubsegmentID = AppendSegment(leftDownward);
      newNewSubsegmentID            = AppendSegment(newSegmentDownward);
      Segment &newLeftSubsegment    = _segments[newLeftSubsegmentID],
              &newNewSubsegment     = _segments[newNewSubsegmentID];
      newNewSubsegment.lowVertex    = lowVertexID;
      newLeftSubsegment.highVertex  = rightNewVertexID;
      newNewSubsegment.highVertex = (leftDownward == newSegmentDownward) ? leftNewVertexID : rightNewVertexID;
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

      AddVertex(leftNewVertexID, leftRegion->nodeID);
      AddVertex(rightNewVertexID, region.nodeID);

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
      return newNewSubsegmentID;
    }
  }

  if (checkRight)
  {
    Segment &rightSegment = _segments[region.right];
    bool rightDownward    = rightSegment.downward;

    VertexID &rightHighVertexID = rightSegment.highVertex, &rightLowVertexID = rightSegment.lowVertex;
    if (Intersected(rightHighVertexID, rightLowVertexID, highVertexID, lowVertexID, &intersection))
    {
      // split vertices
      VertexID leftNewVertexID = AppendVertex(intersection), rightNewVertexID = AppendVertex(intersection);
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

      AddVertex(leftNewVertexID, rightRegion->nodeID);
      AddVertex(rightNewVertexID, region.nodeID);

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
      Region *leftRegion = &region;
      footVertexID       = leftRegion->low;
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
      return newNewSubsegmentID;
    }
  }
  return INVALID_INDEX;
}

void TrapezoidMapP::AssignDepth()
{
  assert(_regions[0].depth == INVALID_DEPTH);  // topmost region
  std::stack<Region *> curStack, nextStack;
  nextStack.push(&_regions[0]);

  Depth curDepth = 0;
  while (!nextStack.empty())
  {
    std::swap(curStack, nextStack);

    // dye all neighbors
    while (!curStack.empty())
    {
      Region *curRegion = curStack.top();
      curStack.pop();

      /* Attention: if the polygon is not closed (bad input), this won't work, we'd use curRegion->depth =
       * std::min(curDepth, curRegion->depth); (and some further modifications have to be performed). But
       * luckily, we don't need to adjust the following logic to adapt to this - i.e. the input polygons are
       * assumed to be closed. */
      if (curRegion->depth != INVALID_DEPTH)
        continue;
      curRegion->depth = curDepth;

      // go across the legs (segments)
      /* due to possible existence of intersections, its undefined to query trapezoid by segments,
         we have to query the left/right neighbor region by the low neighbors of the high vertex of the
         segment. */
      assert(Valid(curRegion->left) && Valid(curRegion->right));
      if (!Infinite(curRegion->left))
      {
        Segment &leftSegment = _segments[curRegion->left];
        RegionID midID       = _lowNeighbors[leftSegment.highVertex].mid;

        RegionID leftNeighborID = Valid(midID) ? midID : _lowNeighbors[leftSegment.highVertex].left;
        Region &leftNeighbor    = _regions[leftNeighborID];
        if (leftNeighbor.depth == INVALID_DEPTH)
          nextStack.push(&leftNeighbor);
      }
      if (!Infinite(curRegion->right))
      {
        Segment &rightSegment = _segments[curRegion->right];
        RegionID midID        = _lowNeighbors[rightSegment.highVertex].mid;

        RegionID rightNeighborID = Valid(midID) ? midID : _lowNeighbors[rightSegment.highVertex].right;
        Region &rightNeighbor    = _regions[rightNeighborID];
        if (rightNeighbor.depth == INVALID_DEPTH)
          nextStack.push(&rightNeighbor);
      }

      // flood
      for (const auto neighborID : {curRegion->highNeighbors[0], curRegion->highNeighbors[1],
                                    curRegion->lowNeighbors[0], curRegion->lowNeighbors[1]})
      {
        if (!Valid(neighborID) || Infinite(neighborID))
          continue;

        Region &neighbor = _regions[neighborID];
        assert(neighbor.depth == INVALID_DEPTH || neighbor.depth == curDepth);

        if (neighbor.depth == INVALID_DEPTH)
          /* sometimes there's only one neighbor, in which occasion [0] will be the same as [1], maybe we
           * should prevent this duplication here (not enabled for now) */
          /* if (curStack.empty() || (curStack.top() != &neighbor)) */
          curStack.push(&neighbor);
      }
    }

    ++curDepth;
  }
}

bool TrapezoidMapP::Higher(VertexID leftVertexID, VertexID rightVertexID) const
{
  const Vertex &leftVertex = _vertices[leftVertexID], &rightVertex = _vertices[rightVertexID];

  int res = Higher(leftVertex, rightVertex);
  if (res > -1)
    return !!res;

  // same x, y: latter vertex always on the right, is this OK?
  // todo: in paper, another random integer is assigned to determine this.
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

int TrapezoidMapP::Higher(const Vertex &refVertex, const Vertex &highVertex, const Vertex &lowVertex) const
{
  auto highLow = highVertex - lowVertex;
  double cross = highLow ^ (refVertex - lowVertex);
  if (cross != 0.)
    return cross > 0;

  return -1;
}

int TrapezoidMapP::Intersected(VertexID segment1_Start,
                               VertexID segment1_End,
                               VertexID segment2_Start,
                               VertexID segment2_End,
                               Vertex *const intersection) const
{
  // todo: return int for intersected at mid/endpoint and not intersected.
  const Vertex &s1 = _vertices[segment1_Start], &e1 = _vertices[segment1_End],
               &s2 = _vertices[segment2_Start], &e2 = _vertices[segment2_End];

  Vec2 vec1 = e1 - s1, vec2 = e2 - s2, s2s1 = s1 - s2;

  double denom = vec1 ^ vec2;
  if (denom == 0)
    return false;  // todo: handle coincidence

  bool denomPositive = denom > 0;

  double s_numer = vec1 ^ s2s1;
  if ((s_numer < 0) == denomPositive)
    return false;  // No collision

  double t_numer = vec2 ^ s2s1;
  if ((t_numer < 0) == denomPositive)
    return false;  // No collision

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
    return false;  // No collision

  // intersected
  if (!intersection)
    return true;

  double t = t_numer / denom;

  intersection->x = s1.x + (t * vec1.x);
  intersection->y = s1.y + (t * vec1.y);

  return true;
}