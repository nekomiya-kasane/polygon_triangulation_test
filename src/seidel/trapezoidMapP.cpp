#pragma once

#include "trapezoidMapP.h"

#include <cassert>
#include <queue>
#include <random>

const double C_PI = 3.1415926535897932;

void TrapezoidMapP::AddPolygon(const Vec2Set &points, bool compactPoints)
{
  AnyID oldSize       = _vertices.Size(),
        incrementSize = static_cast<AnyID>(compactPoints ? points.size() : points.size() - 1);

  _vertices.Reserve(_vertices.Size() + incrementSize);
  _polygonIDs.reserve(_vertices.Size() + incrementSize);

  // todo: memcpy costs too much time for large polygon
  _vertices.Pushback(points.data(), incrementSize);

  for (VertexID i = 0; i < incrementSize; ++i)
    _polygonIDs.push_back(_polygonCount);

  _polygonCount += 1;
  _vertexCount += static_cast<AnyID>(points.size());
}

void TrapezoidMapP::Build()
{
  // fill ends
  _prevVertices.reserve(_vertexCount);
  _endVertices.reserve(_vertexCount);
  _shadowPoints.resize(_vertexCount);
  _sectors.resize(_vertexCount);

  for (AnyID i = 0, s = 0, e = 1; i < _polygonCount; ++i, s = e + 1, e = s + 1)
  {
    // first
    _prevVertices.push_back(0);
    _endVertices.push_back(0);

    // middle
    for (; e != _vertexCount - 1 && _polygonIDs[e + 1] == i; ++e)
    {
      _prevVertices.push_back(e - 1);
      _endVertices.push_back(e + 1);
    }

    // refresh first
    _endVertices[s]  = s + 1;
    _prevVertices[s] = e;

    // last
    _endVertices.push_back(s);
    _prevVertices.push_back(e - 1);

    /* now e is the last and s is the first vertex of current polygon */
  }

  // add segments
  for (VertexID i = 0; i < _vertexCount; ++i)
  {
    // Edge goes from a to b
    VertexID start = i;
    VertexID end   = _endVertices[start];

    SegmentID segmentID = AppendSegment(Higher(start, end));
    Segment &segment    = _segments[segmentID];

    segment.highVertex = segment.downward ? start : end;
    segment.lowVertex  = segment.downward ? end : start;
  }

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
#ifdef _DEBUG
  if (config.incremental)
    _permutation.resize(std::min(static_cast<unsigned int>(_permutation.size()), config.maxSegment));

  if (config.printData)
  {
    std::ios::sync_with_stdio(false);
    for (const auto ID : _permutation)
      std::cout << ID << "-";
    std::cout << std::endl;

    std::cout << "\nVertices: {" << std::endl;
    for (const auto &vertex : _vertices)
    {
      std::cout << "{" << (int)vertex.x << ", " << (int)vertex.y << "}, ";
    }
    std::cout << "}";
  }
#endif

  _vertexRegions.resize(_vertices.Size(), ROOT_NODE_ID);
  _nodes.Reserve(_vertices.Size() * 5 + _segments.Size() * 5 + 6);
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
    config.phase = i - 1;
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
    // for (size_t i = 0; i < _vertexRegions.size(); ++i)
    //{
    //  NodeID &regionNodeID = _vertexRegions[i];
    //  if (!Valid(regionNodeID))
    //    continue;  // already added

    //  regionNodeID = _regions[QueryFrom(regionNodeID, (VertexID)i)].nodeID;
    //}
  }

// assign depth
#ifdef _DEBUG
  if (config.assignDepth)
#endif
    AssignDepth();

#ifdef _DEBUG
  if (config.printData)
  {
    size_t i = 0;
    std::cout << "\nNodes: " << std::endl;
    for (const auto &node : _nodes)
    {
      std::cout << i++ << ": " << node.ToString() << std::endl;
    }
    i = 0;
    std::cout << "\nRegions: " << std::endl;
    for (const auto &region : _regions)
    {
      std::cout << i++ << ": " << region.ToString() << std::endl;
    }
    i = 0;
    std::cout << "\nSegments: " << std::endl;
    for (const auto &segment : _segments)
    {
      std::cout << i++ << ": " << segment.ToString() << std::endl;
    }
    i = 0;
    std::cout << "\nLow Neighbors: " << std::endl;
    for (const auto &lowNeis : _lowNeighbors)
    {
      std::cout << i++ << ": "
                << "(" << lowNeis.left << ", " << lowNeis.mid << ", " << lowNeis.right << ")\n";
    }
    std::cout << std::endl;
  }
#endif
}

void TrapezoidMapP::Reset()
{
  _vertices.Reset();
  _polygonIDs.clear();
  _vertexCount = _polygonCount = 0;
  ClearCache();
}

void TrapezoidMapP::ClearCache()
{
  _nodes.Reset();
  _regions.Reset();
  _lowNeighbors.clear();
  _vertexRegions.clear();
  _permutation.clear();
  _segments.Reset();
  _prevVertices.clear();
  _endVertices.clear();
  _shadowPoints.clear();
  _sectors.clear();
}

bool TrapezoidMapP::AddVertex(VertexID vertexID, NodeID startNodeID)
{
  if (!Finite(_vertexRegions[vertexID]))
  {
    assert(!Infinite(_vertexRegions[vertexID]));
    return true;
  }

  RegionID vertexRegion = QueryFrom(startNodeID, vertexID);
  Node &originalNode    = _nodes[_regions[vertexRegion].nodeID];

  auto highLowRegion = SplitRegionByVertex(vertexRegion, vertexID);
  RegionID highRegionID, lowRegionID;
  std::tie(highRegionID, lowRegionID) = highLowRegion;

  // origin cast to vertex type
  originalNode.type  = Node::VERTEX;
  originalNode.left  = highRegionID;
  originalNode.right = lowRegionID;
  originalNode.value = vertexID;

  _vertexRegions[vertexID] = INVALID_INDEX;  // invalid for already added vertex

  return false;
}

bool TrapezoidMapP::AddSegment(SegmentID segmentID)
{
  TRACE("Add segment: " << segmentID);

  // add from high to low
  Segment &segment = _segments[segmentID];

  bool highAdded = AddVertex(segment.highVertex), lowAdded = AddVertex(segment.lowVertex);

  SegmentID splittedSegment = INVALID_INDEX;

  while (Finite(segmentID))
  {
    int type;
    RegionID originalRegionID = GetFirstIntersectedRegion(segment.highVertex, segment.lowVertex, &type);
    Region &originalRegion    = _regions[originalRegionID];

    if (config.checkIntersection)
      splittedSegment = ResolveIntersection(originalRegionID, segmentID, type != 2, type != -2);

    if (!SplitRegionBySegment(originalRegionID, segmentID, type))
      return false;

    while (Finite(_nextRegion))
    {
      originalRegionID = _nextRegion;
      if (config.checkIntersection)
      {
        splittedSegment = ResolveIntersection(originalRegionID, segmentID, type != 2, type != -2);
      }
      if (!SplitRegionBySegment(originalRegionID, segmentID, type))
        return false;
    }

    segmentID = splittedSegment;
  }

  return !Infinite(segmentID);
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
      node = Lefter(vertexIDtoQuery, node->value) ? &_nodes[node->left] : &_nodes[node->right];
    }
    type = node->type;
  }
  return node->value;
}

VertexID TrapezoidMapP::AppendVertex(const Vertex &vertex)
{
  assert(_vertices.Size() == _endVertices.size());
  assert(_vertices.Size() == _prevVertices.size());
  assert(_vertices.Size() == _vertexRegions.size());

  VertexID id = _vertices.Pushback(vertex);

  _endVertices.push_back(INVALID_INDEX);
  _prevVertices.push_back(INVALID_INDEX);
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
  lowRegion.highNeighbors[0] = regionID;
  lowRegion.highNeighbors[1] = regionID;
  lowRegion.lowNeighbors[0]  = highRegion.lowNeighbors[0];
  lowRegion.lowNeighbors[1]  = highRegion.lowNeighbors[1];

  // update the high neighbor of the lower neighbor of this region
  for (int i = 0; i < 2; ++i)
  {
    RegionID lowNeighborID = highRegion.lowNeighbors[i];
    if (Finite(lowNeighborID))
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
  highRegion.lowNeighbors[0] = lowRegionID;
  highRegion.lowNeighbors[1] = lowRegionID;

  // new NodeID for the original Region since it's now a leaf of the original node
  Node &newNodeForHighRegion = NewNode(Node::REGION, regionID);
  highRegion.nodeID          = newNodeForHighRegion.id;

  // maintain neighbors
  _lowNeighbors[vertexID].left = lowRegionID;

  return {highRegion.nodeID, lowRegion.nodeID};
}

bool TrapezoidMapP::SplitRegionBySegment(RegionID regionID, SegmentID segmentID, int &type)
{
  // The segment won't intersect with the left/right segments of the region, we have ensured this
  // outside this function.
  // two vertices are already inserted, hence we need not to consider other occasions - the vertices
  // must be the high & low vertex of the trapezoid.

  Region &originalRegion = _regions[regionID];
  RegionID highRegionID  = _mergeType == -1 ? _tmpRegionToMerge : regionID;
  Region &highRegion     = _regions[highRegionID];
  Region &lowRegion =
      _mergeType != 0 ? _regions[_mergeType == -1 ? regionID : _tmpRegionToMerge] : NewRegion();
  RegionID lowRegionID = _regions.GetIndex(&lowRegion);

  // update neighbors
  if (!UpdateAbove(originalRegion, highRegion, lowRegion, segmentID, type))
    return false;
  type = UpdateBelow(regionID, highRegionID, lowRegionID, segmentID);

  // assign original NodeID to the overridden node
  Node &originalNode = _nodes[originalRegion.nodeID];
  originalNode.type  = Node::SEGMENT;
  originalNode.value = segmentID;

  Node &newNodeForOriginalRegion = NewNode(Node::Type::REGION, regionID);
  originalRegion.nodeID          = newNodeForOriginalRegion.id;  // renumber high region

  originalNode.left  = highRegion.nodeID;
  originalNode.right = lowRegion.nodeID;

  return true;
}

bool TrapezoidMapP::UpdateAbove(Region &originalRegion,
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
      lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = originalRegion.highNeighbors[1];
      highRegion.highNeighbors[0] = highRegion.highNeighbors[1] = INVALID_INDEX;
      lowRegion.high                                            = originalRegion.high;

      // update _lowNeighbors
      assert(!Invalid(highVertLowNei.right));
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
      lowRegion.high                                          = originalRegion.high;

      // update _lowNeighbors
      assert(Invalid(highVertLowNei.mid) && (highVertLowNei.left == _regions.GetIndex(&highRegion)));
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
      assert(Invalid(lowRegion.highNeighbors[0]) && Invalid(lowRegion.highNeighbors[1]));
      lowRegion.high = originalRegion.high;

      // update _lowNeighbors
      assert(highVertLowNei.left == _regions.GetIndex(&highRegion));
      highVertLowNei.mid = lowRegionID;
      break;
    }

    // occasion 1:    .                 occasion 2:                 .
    //   |-.-.-.-.\---*------------|      |-.-.-.-.\----------------*------------|
    //   |  High   \               |      |  High   \               |            |
    //   | (Merge)  \     Low      |      | (Merge)  \     Low      |            |
    //   |           \ <--- added  |      |           \ <--- added  |            |
    case -1:
    {
      // update this region's high neighbor's low neighbors
      Region &highNeiRegion         = _regions[originalRegion.highNeighbors[1]];
      bool occasion1                = highNeiRegion.lowNeighbors[0] == highNeiRegion.lowNeighbors[1];
      highNeiRegion.lowNeighbors[0] = lowRegionID;
      if (occasion1)
        highNeiRegion.lowNeighbors[1] = lowRegionID;

      // update this region's neighbors
      lowRegion.highNeighbors[0] = originalRegion.highNeighbors[0];
      lowRegion.highNeighbors[1] = originalRegion.highNeighbors[1];
      // wrong!! -> highRegion.highNeighbors[1] = highRegion.highNeighbors[0];

      // update _lowNeighbors
      highVertLowNei.left = lowRegionID;
      break;
    }

    //               .                       .
    //   |-----------*-----/-.-.-.-|      |--*----------------/-.-.-.-|
    //   |    High        /  Low   |      |  |        High   /  Low   |
    //   |               / (Merge) |  or  |  |              / (Merge) |
    //   |   added ---> /          |      |  |  added ---> /          |
    case 1:
    {
      // wrong!! -> lowRegion.highNeighbors[0] = lowRegion.highNeighbors[1] = highRegion.highNeighbors[1];
      // nothing to do actually.
      break;
    }
  }

  // update left and right segment
  lowRegion.right = originalRegion.right;
  lowRegion.left = highRegion.right = segmentID;

  // We modify high neighbors and segments in this method, but don't modify low neighbors and vertices
  // Whence its safe to query high/low vertices and low neighbors on highRegion in `UpdateBelow`.
  return true;
}

int TrapezoidMapP::UpdateBelow(RegionID originalRegionID,
                               RegionID highRegionID,
                               RegionID lowRegionID,
                               SegmentID segmentID)
{
  assert(Finite(segmentID) && Finite(originalRegionID) && Finite(highRegionID) && Finite(lowRegionID));

  // merging already completed before calling this.
  Region &originalRegion = _regions[originalRegionID], &highRegion = _regions[highRegionID],
         &lowRegion = _regions[lowRegionID];
  Segment &segment  = _segments[segmentID];
  // high left is the left of original region, low right is the right of original region
  VertexID leftLowVertex  = !Infinite(highRegion.left) ? _segments[highRegion.left].lowVertex : INVALID_INDEX,
           rightLowVertex = !Infinite(lowRegion.right) ? _segments[lowRegion.right].lowVertex : INVALID_INDEX;

  // the last region
  if (originalRegion.low == segment.lowVertex)
  {
    if (leftLowVertex == segment.lowVertex)
    {
      //  \..../............|
      //   \../.<---.added..|
      //    \/..............|
      //-----*--------------|
      //                    |

      // update this region's low neighbor's high neighbors
      Region &lowNeiRegion          = _regions[originalRegion.lowNeighbors[1]];
      lowNeiRegion.highNeighbors[1] = lowRegionID;

      // update this region's neighbors
      lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = originalRegion.lowNeighbors[1];
      highRegion.lowNeighbors[0] = highRegion.lowNeighbors[1] = INVALID_INDEX;
      highRegion.low = lowRegion.low = leftLowVertex;
    }
    else if (rightLowVertex == segment.lowVertex)
    {
      //  |...........\..../
      //  |.added.--->.\../
      //  |.............\/
      //  |-------------*---
      //  |

      // update this region's low neighbor's high neighbors
      Region &lowNeiRegion          = _regions[originalRegion.lowNeighbors[0]];
      lowNeiRegion.highNeighbors[0] = highRegionID;

      // update this region's neighbors
      highRegion.lowNeighbors[0] = highRegion.lowNeighbors[1] = originalRegion.lowNeighbors[0];
      lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = INVALID_INDEX;
      highRegion.low = lowRegion.low = rightLowVertex;
    }
    else
    {
      // low neighbors of the low vertex of the original region
      // todo: what about use the low neighbors stored in originalRegion
      const auto &oriLowVertLowNei = _lowNeighbors[originalRegion.low];
      Region &lowNeiRegionLow      = _regions[originalRegion.lowNeighbors[1]],
             &lowNeiRegionHigh     = _regions[originalRegion.lowNeighbors[0]];

      highRegion.lowNeighbors[0] = highRegion.lowNeighbors[1] = oriLowVertLowNei.left;
      highRegion.low = lowRegion.low = originalRegion.low;

      if (Finite(oriLowVertLowNei.right))
      {
        //  |...........\.....|
        //  |.added.--->.\....|
        //  |.............\...|
        //  |-------------*---|
        //  |            /    |

        // update this region's neighbors
        assert(oriLowVertLowNei.Size() == 2);
        lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = oriLowVertLowNei.right;

        // update this region's low neighbor's high neighbors

        lowNeiRegionLow.highNeighbors[0] = lowNeiRegionLow.highNeighbors[1] = lowRegionID;
        lowNeiRegionHigh.highNeighbors[0] = lowNeiRegionHigh.highNeighbors[1] = highRegionID;
      }
      else
      {
        //  |...........\.....|
        //  |.added.--->.\....|
        //  |.............\...|
        //  |-------------*---|
        //  |                 |

        // update this region's neighbors
        lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = oriLowVertLowNei.left;

        // update this region's low neighbor's high neighbors
        lowNeiRegionLow.highNeighbors[0] = highRegionID;
        lowNeiRegionLow.highNeighbors[1] = lowRegionID;
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
    int res = Lefter(originalRegion.low, segmentID) ? 1 : -1;
    if (res == 1)  // to low right
    {
      // clang-format off
      // occasion 1:              occasion 2:                 occasion 3:                occasion 4:
      //   |           /      |                    /      |                    /      |    .  |           /      | if mergeType = -1
      //   |   high   /  low  |        high       /  low  |         high      /  low  |     . |   high   /  low  |    then low is old
      //   |         /  merge |                  /  merge |                  /  merge |      .|         /  merge | if mergeType = 0
      //   *--------+vvvvvvvvv|     ---*--------+vvvvvvvvv|     ---*--------+vvvvvvvvv|    ---*--------+vvvvvvvvv|    then low is newed
      //  /  next  +          |          next  +          |       .|  next +          |         next  +          |
      // /        +           |               +           |      . |      +           |              +           |
      // clang-format on

      if (_mergeType == -1)
      // condition not necessary, but if _mergeType != -1, then the following items are not changed
      {
        Region &lowNeiRegionLow = _regions[originalRegion.lowNeighbors[1]];
        if (lowNeiRegionLow.highNeighbors[0] == lowNeiRegionLow.highNeighbors[1])  // occasion 1/2/3
          lowNeiRegionLow.highNeighbors[0] = highRegionID;
        if (originalRegion.lowNeighbors[0] != originalRegion.lowNeighbors[1])  // occasion 3
        {
          Region &lowNeiRegionHigh          = _regions[originalRegion.lowNeighbors[0]];
          lowNeiRegionHigh.highNeighbors[0] = lowNeiRegionHigh.highNeighbors[1] = highRegionID;
        }

        lowNeiRegionLow.highNeighbors[1] = highRegionID;

        highRegion.lowNeighbors[0] = originalRegion.lowNeighbors[0];
        highRegion.lowNeighbors[1] = originalRegion.lowNeighbors[1];
      }

      _nextRegion       = originalRegion.lowNeighbors[1];
      _tmpRegionToMerge = lowRegionID;
      _mergeType        = 1;  // merge low subregion

      // update current region
      lowRegion.lowNeighbors[0] = lowRegion.lowNeighbors[1] = originalRegion.lowNeighbors[1];
      highRegion.low                                        = originalRegion.low;
    }
    else
    {
      // clang-format off
      // occasion 1:                     occasion 2:                     occasion 3:
      //   | (old) \   （new)   |           | (old) \   （new)   |  .        | (old) \      （new)      | (old) \      （new)
      //   |  high  \   low    |           |  high  \   low    | .         |  high  \      low       |  high  \      low
      //   |  merge  \         |           |  merge  \         |.          |  merge  \               |  merge  \            
      //   |vvvvvvvvvv+--------*     or    |vvvvvvvvvv+--------*---        |vvvvvvvvvv+------*----   |vvvvvvvvvv+------*----
      //   |           +        \          |           +                   |           +     |.      |           +     
      //   |            +        \         |            +                  |            +    | .     |            +
      // clang-format on

      // if (_mergeType == 1) needed？

      _nextRegion       = originalRegion.lowNeighbors[0];
      _tmpRegionToMerge = highRegionID;
      _mergeType        = -1;  // merge high subregion

      // update the region below current region
      Region &lowNeiRegionHigh = _regions[originalRegion.lowNeighbors[0]];

      /* occasion 1 & 3 */ bool oneHighNeiForRegionBelow =
          lowNeiRegionHigh.highNeighbors[0] == lowNeiRegionHigh.highNeighbors[1];
      if (oneHighNeiForRegionBelow)
      {
        lowNeiRegionHigh.highNeighbors[1] = lowRegionID;
      }
      lowNeiRegionHigh.highNeighbors[0] = lowRegionID;

      /* occasion 3 */ bool twoLowNeiForThisRegion =
          originalRegion.lowNeighbors[0] != originalRegion.lowNeighbors[1];
      if (twoLowNeiForThisRegion)
      {
        Region &lowNeiRegionLow          = _regions[originalRegion.lowNeighbors[1]];
        lowNeiRegionLow.highNeighbors[0] = lowNeiRegionLow.highNeighbors[1] = lowRegionID;
      }

      // update current region
      lowRegion.lowNeighbors[0]  = originalRegion.lowNeighbors[0];
      lowRegion.lowNeighbors[1]  = originalRegion.lowNeighbors[1];
      lowRegion.low              = originalRegion.low;
      highRegion.lowNeighbors[1] = highRegion.lowNeighbors[0];  // needed?
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
      // const Segment &midSegment = _segments[_regions[lowNeighbors.left].right];
      if (Lefter(refVertex, _regions[lowNeighbors.left].right))
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
  // CAUTION! new vertices and segments may be introduced here, any references and pointers may be
  // invalidated, only IDs are safe.
  return INVALID_INDEX;

  VertexID highVertexID, lowVertexID;
  SegmentID leftSegmentID, rightSegmentID;
  bool newSegmentDownward;
  {
    Segment &newSegment = _segments[segmentID];
    highVertexID        = _segments[segmentID].highVertex;
    lowVertexID         = _segments[segmentID].lowVertex;
    leftSegmentID       = _regions[curRegionID].left;
    rightSegmentID      = _regions[curRegionID].right;
    newSegmentDownward  = newSegment.downward;
  }

  Vertex intersection;
  if (checkLeft && !Infinite(leftSegmentID))
  {
    bool leftDownward;
    VertexID leftHighVertexID, leftLowVertexID;
    assert(Finite(leftSegmentID));
    {
      Segment &leftSegment = _segments[leftSegmentID];
      leftDownward         = leftSegment.downward;
      leftHighVertexID = leftSegment.highVertex, leftLowVertexID = leftSegment.lowVertex;
    }
    if (Intersected(leftHighVertexID, leftLowVertexID, highVertexID, lowVertexID, &intersection))
    {
      // split vertices
      VertexID leftNewVertexID = AppendVertex(intersection), rightNewVertexID = AppendVertex(intersection);
      SegmentID newLeftSubsegmentID = AppendSegment(leftDownward),
                newNewSubsegmentID  = AppendSegment(newSegmentDownward);

      _segments[newNewSubsegmentID].lowVertex   = lowVertexID;
      _segments[newLeftSubsegmentID].highVertex = rightNewVertexID;
      _segments[newNewSubsegmentID].highVertex =
          (leftDownward == newSegmentDownward) ? rightNewVertexID : leftNewVertexID;
      _segments[newLeftSubsegmentID].lowVertex = leftLowVertexID;

      // find left intersected region
      RegionID leftRegionID = Finite(_lowNeighbors[leftHighVertexID].mid)
                                  ? _lowNeighbors[leftHighVertexID].mid
                                  : _lowNeighbors[leftHighVertexID].left;
      {
        Region *leftRegion = &_regions[leftRegionID];
        while (true)
        {
          RegionID leftBelowRegionID = leftRegion->lowNeighbors[1];  // right most
          Region &leftBelowRegion    = _regions[leftBelowRegionID];

          if (!Infinite(leftBelowRegion.low) /* && Higher(_vertices[leftBelowRegion.low], intersection)*/)
          {
            leftRegionID = leftBelowRegionID;
            leftRegion   = &leftBelowRegion;
            continue;
          }
          break;
        }

        AddVertex(leftNewVertexID, leftRegion->nodeID);
        AddVertex(rightNewVertexID, _regions[curRegionID].nodeID);
      }

      Region &region      = _regions[curRegionID];
      Segment &newSegment = _segments[segmentID];

      // resolve left regions
      {
        Region *leftRegion = &_regions[leftRegionID];
        // VertexID footVertexID;
        do
        {
          leftRegionID = leftRegion->lowNeighbors[1];
          leftRegion   = &_regions[leftRegionID];

          // modify right segment
          if (leftRegion->left == region.left)
            leftRegion->right = newLeftSubsegmentID;
          else
            break;
        } while (true);
      }

      // resolve right regions
      {
        Region *rightRegion    = &region;
        RegionID rightRegionID = curRegionID;
        do
        {
          rightRegionID = rightRegion->lowNeighbors[0];
          rightRegion   = &_regions[leftRegionID];

          // modify right segment
          if (rightRegion->left == region.left)
            rightRegion->left = newLeftSubsegmentID;
          else
            break;
        } while (true);
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

  if (checkRight && !Infinite(_regions[curRegionID].right))
  {
    bool rightDownward;
    VertexID rightHighVertexID, rightLowVertexID;
    assert(Finite(rightSegmentID) && Finite(_regions[curRegionID].right));
    {
      Segment &rightSegment = _segments[rightSegmentID];
      rightDownward         = rightSegment.downward;
      rightHighVertexID = rightSegment.highVertex, rightLowVertexID = rightSegment.lowVertex;
    }
    if (Intersected(rightHighVertexID, rightLowVertexID, highVertexID, lowVertexID, &intersection))
    {
      // split vertices
      VertexID leftNewVertexID = AppendVertex(intersection), rightNewVertexID = AppendVertex(intersection);
      SegmentID newRightSubsegmentID = AppendSegment(rightDownward),
                newNewSubsegmentID   = AppendSegment(newSegmentDownward);

      _segments[newRightSubsegmentID].highVertex =
          (rightDownward == newSegmentDownward) ? leftNewVertexID : rightNewVertexID;
      _segments[newNewSubsegmentID].highVertex  = rightNewVertexID;
      _segments[newNewSubsegmentID].lowVertex   = lowVertexID;
      _segments[newRightSubsegmentID].lowVertex = rightLowVertexID;

      // find right intersected region
      RegionID rightRegionID = Finite(_lowNeighbors[rightHighVertexID].mid)
                                   ? _lowNeighbors[rightHighVertexID].mid
                                   : _lowNeighbors[rightHighVertexID].right;
      {
        Region *rightRegion = &_regions[rightRegionID];
        while (true)
        {
          RegionID rightBelowRegionID = rightRegion->lowNeighbors[0];  // left most
          Region &rightBelowRegion    = _regions[rightBelowRegionID];

          if (!Infinite(rightBelowRegion.low) /* && Higher(_vertices[rightBelowRegion.low], intersection)*/)
          {
            rightRegionID = rightBelowRegionID;
            rightRegion   = &rightBelowRegion;
            continue;
          }
          break;
        }

        AddVertex(leftNewVertexID, _regions[curRegionID].nodeID);
        AddVertex(rightNewVertexID, rightRegion->nodeID);
      }

      Region &region      = _regions[curRegionID];
      Segment &newSegment = _segments[segmentID];

      // resolve right regions
      {
        Region *rightRegion = &_regions[rightRegionID];
        // VertexID footVertexID;
        do
        {
          rightRegionID = rightRegion->lowNeighbors[0];
          rightRegion   = &_regions[rightRegionID];

          // modify right segment
          if (rightRegion->left == region.right)
            rightRegion->left = newRightSubsegmentID;
          else
            break;
        } while (true);
      }

      // resolve left regions
      {
        Region *leftRegion    = &region;
        RegionID leftRegionID = curRegionID;
        do
        {
          leftRegionID = leftRegion->lowNeighbors[1];
          leftRegion   = &_regions[leftRegionID];

          // modify right segment
          if (leftRegion->right == region.right)
            leftRegion->right = newRightSubsegmentID;
          else
            break;
        } while (true);
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

      newSegment.lowVertex                = leftNewVertexID;
      _segments[rightSegmentID].lowVertex = rightNewVertexID;
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
      assert(!Invalid(curRegion->left) && !Invalid(curRegion->right));
      if (!Infinite(curRegion->left))
      {
        Segment &leftSegment = _segments[curRegion->left];
        RegionID midID       = _lowNeighbors[leftSegment.highVertex].mid;

        RegionID leftNeighborID = Finite(midID) ? midID : _lowNeighbors[leftSegment.highVertex].left;
        Region &leftNeighbor    = _regions[leftNeighborID];
        if (leftNeighbor.depth == INVALID_DEPTH)
          nextStack.push(&leftNeighbor);
      }
      if (!Infinite(curRegion->right))
      {
        Segment &rightSegment = _segments[curRegion->right];
        RegionID midID        = _lowNeighbors[rightSegment.highVertex].mid;

        RegionID rightNeighborID = Finite(midID) ? midID : _lowNeighbors[rightSegment.highVertex].right;
        Region &rightNeighbor    = _regions[rightNeighborID];
        if (rightNeighbor.depth == INVALID_DEPTH)
          nextStack.push(&rightNeighbor);
      }

      // flood
      for (const auto neighborID : {curRegion->highNeighbors[0], curRegion->highNeighbors[1],
                                    curRegion->lowNeighbors[0], curRegion->lowNeighbors[1]})
      {
        if (!Finite(neighborID))
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

bool AngleInSector(double s, double e, double test)
{
  if (e < 0 && s > 0)
  {
    e += C_PI;
    if (test < 0)
      test += C_PI;
  }
  return (test >= s && test < e) || (test > s && test <= e);
}

bool TrapezoidMapP::Higher(VertexID leftVertexID, VertexID rightVertexID) const
{
  const Vertex &leftVertex = _vertices[leftVertexID], &rightVertex = _vertices[rightVertexID];

  if (leftVertex.y != rightVertex.y)
    return leftVertex.y > rightVertex.y;

  if (leftVertex.x != rightVertex.x)
    return leftVertex.x < rightVertex.x;

  const Vertex &leftShadow  = const_cast<TrapezoidMapP *>(this)->GetShadowPoint(leftVertexID),
               &rightShadow = const_cast<TrapezoidMapP *>(this)->GetShadowPoint(rightVertexID);

  auto As = _sectors[leftVertexID].first, Ae = _sectors[leftVertexID].second;
  auto Bs = _sectors[rightVertexID].first, Be = _sectors[rightVertexID].second;

  bool AInB = AngleInSector(Bs, Be, As) && AngleInSector(Bs, Be, Ae);
  if (AInB)
  {
    if (leftShadow.y != 0)
      return leftShadow.y > 0;
    return leftShadow.x < 0;
  }

  bool BInA = AngleInSector(Bs, Be, As) && AngleInSector(Bs, Be, Ae);
  if (BInA)
  {
    if (rightShadow.y != 0)
      return rightShadow.y > 0;
    return rightShadow.x < 0;
  }

  if (leftShadow.y != rightShadow.y)
    return leftShadow.y > rightShadow.y;

  if (leftShadow.x != rightShadow.x)
    return leftShadow.x < rightShadow.x;

  assert(false);
  return true;
}

bool TrapezoidMapP::Lefter(VertexID refVertexID, SegmentID segmentID) const
{
  const Segment &segment = _segments[segmentID];
  VertexID highVertexID = segment.highVertex, lowVertexID = segment.lowVertex;
  const Vertex &highVertex = _vertices[highVertexID], &lowVertex = _vertices[lowVertexID],
               &refVertex = _vertices[refVertexID];

  double cross = (highVertex - lowVertex) ^ (refVertex - lowVertex);

  if (cross != 0.)
    return cross > 0.;

  const Vertex &shadowPoint = const_cast<TrapezoidMapP *>(this)->GetShadowPoint(refVertexID);
  cross                     = (highVertex - lowVertex) ^ (shadowPoint - lowVertex);
  assert(cross != 0.);
  return cross > 0.;
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

const Vertex &TrapezoidMapP::GetShadowPoint(VertexID vertexID)
{
  Vertex &shadowPoint = _shadowPoints[vertexID];
  if (shadowPoint == Vec2::origin)
  {
    Vec2 A = _vertices[_prevVertices[vertexID]] - _vertices[vertexID];
    Vec2 B = _vertices[_endVertices[vertexID]] - _vertices[vertexID];

    double cross = (A ^ B);

    if (cross == 0.)  // collinear
    {
      shadowPoint.x = -B.y;
      shadowPoint.y = -B.x;
      shadowPoint /= shadowPoint.Norm();
    }
    else
    {
      shadowPoint = (A + B) / (A + B).Norm();
    }

    auto &sector = _sectors[vertexID];
    sector       = std::make_pair(std::atan2(A.y, A.x), std::atan2(B.y, B.x));
    if (cross < 0)
      std::swap(sector.first, sector.second);
  }

  return shadowPoint;
}