#include "triangulator.h"

#include <cassert>

Mountains Triangulator::ExtractMountains()
{
  // todo: simplify the code

  std::vector<char> leftSegmentVisited(_regions.Size(), false),
      rightSegmentVisited(_regions.Size(), false);

  const auto TriangulateMountain = [this](const Mountain &mountain) -> Triangles {
    if (config.earClippingInsteadOfChimneyClipping)
      return EarClipping(mountain);
    return ChimneyClipping(mountain);
  };

  for (RegionID regionID = 0, n = _regions.Size(); regionID < n; ++regionID)
  {
    Region &region = _regions[regionID];
    if (region.depth % 2 == 0)
      continue;

    // resolve triangle started region
    if (leftSegmentVisited[regionID] && rightSegmentVisited[regionID])  // bad
      continue;

    const SegmentID leftSegmentID = region.left, rightSegmentID = region.right;
    const Segment &leftSegment = _segments[leftSegmentID], rightSegment = _segments[rightSegmentID];

    if ((region.high == leftSegment.highVertex && region.low == leftSegment.lowVertex) ||
        (region.high == rightSegment.highVertex && region.low == rightSegment.lowVertex))
      continue;

    // left segment not visited
    if (!leftSegmentVisited[regionID])
    {
      VertexID baseVertexID = leftSegment.highVertex;

      Mountain mountain;
      // jump to the top then go down
      if (config.useNeighborCacheToTransverse)
      {
        mountain.push_back(leftSegment.lowVertex);
        mountain.push_back(baseVertexID);

        const auto &tln = _lowNeighbors[baseVertexID];  // top low neighbors
        Region *curRegionPtr =
            _regions[tln.right].left == leftSegmentID ? &_regions[tln.right] : &_regions[tln.mid];

        while (curRegionPtr->high != leftSegment.lowVertex)
        {
          if (mountain.empty() || mountain.back() != curRegionPtr->high)
            mountain.push_back(curRegionPtr->high);

          leftSegmentVisited[_regions.GetIndex(curRegionPtr)] = true;

          RegionID nextRegionID = curRegionPtr->lowNeighbors[0];
          assert(Valid(nextRegionID));
          if (!Valid(nextRegionID))
            break;
          curRegionPtr = &_regions[nextRegionID];
        }
      }
      // go down then transverse up
      else
      {
        Region *curRegionPtr = &region, *lastRegionPtr = curRegionPtr;

        mountain.push_back(baseVertexID);
        mountain.push_back(leftSegment.lowVertex);

        // go down to the lowest region with the same left segment
        while (_segments[curRegionPtr->left].highVertex == baseVertexID)
        {
          lastRegionPtr = curRegionPtr;

          leftSegmentVisited[_regions.GetIndex(lastRegionPtr)] = true;

          RegionID nextRegionID = curRegionPtr->lowNeighbors[0];
          assert(Valid(nextRegionID));
          if (!Valid(nextRegionID))
            break;

          curRegionPtr = &_regions[nextRegionID];
        }

        // go up to transverse the mountain
        curRegionPtr = lastRegionPtr;
        while (curRegionPtr->high != baseVertexID)
        {
          assert(Valid(curRegionPtr->high));
          if (!Valid(curRegionPtr->high))
            break;

          mountain.push_back(curRegionPtr->high);

          RegionID nextRegionID = curRegionPtr->highNeighbors[0];
          assert(Valid(nextRegionID));
          if (!Valid(nextRegionID))
            break;

          leftSegmentVisited[nextRegionID] = true;  // todo: should this be right?

          curRegionPtr = &_regions[nextRegionID];
        }
      }

      TriangulateMountain(mountain);
    }

    // right segment not visited
    if (!rightSegmentVisited[regionID])
    {
      VertexID baseVertexID = rightSegment.highVertex;

      Mountain mountain;
      // jump to the top then go down
      if (config.useNeighborCacheToTransverse)
      {
        mountain.push_back(rightSegment.lowVertex);
        mountain.push_back(baseVertexID);

        const auto &tln = _lowNeighbors[baseVertexID];  // top low neighbors
        Region *curRegionPtr =
            _regions[tln.left].left == rightSegmentID ? &_regions[tln.left] : &_regions[tln.mid];

        while (curRegionPtr->high != rightSegment.lowVertex)
        {
          if (mountain.empty() || mountain.back() != curRegionPtr->high)
            mountain.push_back(curRegionPtr->high);

          rightSegmentVisited[_regions.GetIndex(curRegionPtr)] = true;

          RegionID nextRegionID = curRegionPtr->lowNeighbors[0];
          assert(Valid(nextRegionID));
          if (!Valid(nextRegionID))
            break;
          curRegionPtr = &_regions[nextRegionID];
        }
      }
      // go down then transverse up
      else
      {
        Region *curRegionPtr = &region, *lastRegionPtr = curRegionPtr;

        mountain.push_back(baseVertexID);
        mountain.push_back(leftSegment.lowVertex);

        // go down to the lowest region with the same left segment
        while (_segments[curRegionPtr->left].highVertex == baseVertexID)
        {
          lastRegionPtr = curRegionPtr;

          leftSegmentVisited[_regions.GetIndex(lastRegionPtr)] = true;

          RegionID nextRegionID = curRegionPtr->lowNeighbors[0];
          assert(Valid(nextRegionID));
          if (!Valid(nextRegionID))
            break;

          curRegionPtr = &_regions[nextRegionID];
        }

        // go up to transverse the mountain
        curRegionPtr = lastRegionPtr;
        while (curRegionPtr->high != baseVertexID)
        {
          assert(Valid(curRegionPtr->high));
          if (!Valid(curRegionPtr->high))
            break;

          mountain.push_back(curRegionPtr->high);

          RegionID nextRegionID = curRegionPtr->highNeighbors[0];
          assert(Valid(nextRegionID));
          if (!Valid(nextRegionID))
            break;

          leftSegmentVisited[nextRegionID] = true;

          curRegionPtr = &_regions[nextRegionID];
        }
      }

      TriangulateMountain(mountain);
    }

    // assert(!(leftSegmentVisited[regionID] && rightSegmentVisited[regionID]));
    // if (leftSegmentVisited[regionID] && rightSegmentVisited[regionID])  // bad
    //   continue;

    //// one-side trapezoid or degenerated trapezoid, but may be in a mountain that start (from top)
    //// from a degenerated trapezoid
    // if (_endVertices[region.high] == region.low || _endVertices[region.high] == region.low)
    //   continue;

    //// extract left based segments
    // Mountain mountain;
    // if (!leftSegmentVisited[regionID])
    //{
    //   Region *current = &_regions[regionID], *last = current;
    //   const Segment &leftSegment = _segments[current->left];
    //   VertexID baseVertexID      = leftSegment.highVertex;

    //  // add base segment's endpoints
    //  mountain.push_back(leftSegment.lowVertex);
    //  mountain.push_back(baseVertexID);

    //  if (config.useNeighborCacheToTransverse)
    //  {
    //    const auto &tln = _lowNeighbors[baseVertexID];  // top low neighbors
    //    current =
    //        _regions[tln.right].left == current->left ? &_regions[tln.right] : &_regions[tln.mid];
    //    last = current;

    //    if ()
    //  }
    //  else
    //  {
    //    // add right segments' endpoints
    //    while (_segments[current->left].highVertex ==)
    //    {}
    //  }
    //}
  }
}

bool Triangulator::TryDiagonalize(Region *regionPtr)
{
  ContourVertex *prev = vertices.PrevWrap(curr);
  ContourVertex *next = vertices.NextWrap(curr);

  Vec2 p1 = vertexBuffer[prev->Index];
  Vec2 p2 = vertexBuffer[curr->Index];
  Vec2 p3 = vertexBuffer[next->Index];

  return Cross(p2 - p1, p3 - p1) > 0.000001f;
}

bool