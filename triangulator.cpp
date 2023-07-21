#include "triangulator.h"

#include <cassert>

Mountains Triangulator::ExtractMountains() const
{
  // todo: simplify the code

  std::vector<char> leftSegmentVisited(_regions.Size(), false),
      rightSegmentVisited(_regions.Size(), false);

  Mountains mountains;

  const auto ProcedureLeft = [this, &leftSegmentVisited, &rightSegmentVisited, &mountains](
                                 const Region &region, SegmentID leftSegmentID,
                                 const Segment &leftSegment) {
    VertexID baseVertexID = leftSegment.highVertex;

    Mountain mountain;  // first and last vertex be the endpoints of the base egde
    // jump to the top then go down
    if (config.useNeighborCacheToTransverse)
    {
      mountain.push_back(baseVertexID);

      const auto &tln = _lowNeighbors[baseVertexID];  // top low neighbors
      const Region *curRegionPtr =
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

      mountain.push_back(leftSegment.lowVertex);
    }
    // go down then transverse up
    else
    {
      const Region *curRegionPtr = &region, *lastRegionPtr = curRegionPtr;

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

      mountain.push_back(baseVertexID);
    }

    mountains.push_back(mountain);
  };

  const auto ProcedureRight = [this, &leftSegmentVisited, &rightSegmentVisited, &mountains](
                                  const Region &region, SegmentID rightSegmentID,
                                  const Segment &rightSegment) {
    VertexID baseVertexID = rightSegment.highVertex;

    Mountain mountain;
    // jump to the top then go down
    if (config.useNeighborCacheToTransverse)
    {
      mountain.push_back(baseVertexID);

      const auto &tln = _lowNeighbors[baseVertexID];  // top low neighbors
      const Region *curRegionPtr =
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

      mountain.push_back(rightSegment.lowVertex);
    }
    // go down then transverse up
    else
    {
      const Region *curRegionPtr = &region, *lastRegionPtr = curRegionPtr;

      mountain.push_back(rightSegment.lowVertex);

      // go down to the lowest region with the same left segment
      while (_segments[curRegionPtr->left].highVertex == baseVertexID)
      {
        lastRegionPtr = curRegionPtr;

        rightSegmentVisited[_regions.GetIndex(lastRegionPtr)] = true;

        RegionID nextRegionID = curRegionPtr->lowNeighbors[1];
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

        RegionID nextRegionID = curRegionPtr->highNeighbors[1];
        assert(Valid(nextRegionID));
        if (!Valid(nextRegionID))
          break;

        rightSegmentVisited[nextRegionID] = true;

        curRegionPtr = &_regions[nextRegionID];
      }

      mountain.push_back(baseVertexID);
    }

    mountains.push_back(mountain);
  };

  // resolve normal mountain
  for (RegionID regionID = 0, n = _regions.Size(); regionID < n; ++regionID)
  {
    const Region &region = _regions[regionID];
    if (region.depth % 2 == 0)
      continue;

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
      ProcedureLeft(region, leftSegmentID, leftSegment);
    }

    // right segment not visited
    if (!rightSegmentVisited[regionID])
    {
      ProcedureRight(region, rightSegmentID, rightSegment);
    }
  }

  // resolve remained degenerated trapezoids
  for (RegionID regionID = 0, n = _regions.Size(); regionID < n; ++regionID)
  {
    if (leftSegmentVisited[regionID] || rightSegmentVisited[regionID])
      continue;

    const Region &region = _regions[regionID];
    if (region.depth % 2 == 0)
      continue;

    const SegmentID leftSegmentID = region.left, rightSegmentID = region.right;
    const Segment &leftSegment = _segments[leftSegmentID], rightSegment = _segments[rightSegmentID];
    if (leftSegment.highVertex != rightSegment.highVertex)
      continue;

    if (!Higher(leftSegment.lowVertex, rightSegment.lowVertex))
    {
      ProcedureLeft(region, leftSegmentID, leftSegment);
    }
    else
    {
      ProcedureRight(region, rightSegmentID, rightSegment);
    }
  }
}

Triangles Triangulator::TriangulateMountain(const Mountain &mountain, Triangles &out) const
{
  if (config.earClippingInsteadOfChimneyClipping)
    return EarClipping(mountain, out);
  return ChimneyClipping(mountain, out);
}

Triangles Triangulator::Triangulate() const
{
  Triangles triangles;

  for (const auto &mountain : ExtractMountains())
    TriangulateMountain(mountain, triangles);

  return triangles;
}

Triangles Triangulator::EarClipping(const Mountain &mountain, Triangles &out) const
{
  // [Monotone Mountain Triangulation] using a special ear clipping algorithm:
  //   Initialize an empty list
  //   Add all convex vertices, excluding the endpoints of the base, to the list
  //   While list is not empty
  //     Cut off the corresponding ear of the monotone mountain
  //     Delete the vertex from the list
  //     For the two neighbors of the vertex
  //       if the neighboring vertex is not an endpoint of the base, and was made convex by cutting
  //       off the ear, then add this neighbor to the list

  std::vector<VertexID> prevs, nexts, current;
  prevs.reserve(mountain.size());
  nexts.reserve(mountain.size());
  size_t n = mountain.size();

  VertexID family[3];
  for (VertexID i = 0; i < n; ++i)
  {
    prevs.push_back(i ? i - 1 : n - 1);
    nexts.push_back(i == n - 1 ? 0 : i + 1);

    family[0] = prevs[i];
    family[1] = i;
    family[2] = nexts[i];

    if (IsConvex(family))
    {
      current.push_back(i);
    }
  }

  VertexID thisVertex = current.back(), base1 = mountain.front(), base2 = mountain.back();
  while (n)
  {
    VertexID prev = prevs[thisVertex], next = nexts[thisVertex];
    out.push_back(Triangle{_vertices[prev], _vertices[thisVertex], _vertices[next]});

    prevs[next]       = prev;
    nexts[prev]       = next;
    prevs[thisVertex] = nexts[thisVertex] = INVALID_INDEX;

    current.pop_back();

    // prev neighbor
    family[0] = prevs[prev];
    family[1] = prev;
    family[2] = nexts[prev];
    if (base1 != prev && base2 != prev && IsConvex(family))
      current.push_back(prev);

    // next neighbor
    family[0] = prevs[next];
    family[1] = next;
    family[2] = nexts[next];
    if (base1 != next && base2 != next && IsConvex(family))
      current.push_back(next);
  }

  return out;
}

Triangles Triangulator::ChimneyClipping(const Mountain &mountain, Triangles &out) const
{
  assert(mountain.size() > 2);
  std::vector<VertexID> stack = {mountain[0], mountain[1]};

  VertexID next;
  for (size_t i = 2; i < mountain.size() && stack.size() >= 2; ++i)
  {
    next                  = mountain[i];
    VertexID prevThree[3] = {next, stack.back(), *(stack.cend() - 2)};
    if (!IsConvex(prevThree))
      stack.push_back(next);
    else
    {
      while (IsConvex(prevThree))
      {
        if (!IsZeroSize(prevThree))
          out.push_back(
              Triangle{_vertices[prevThree[2]], _vertices[prevThree[1]], _vertices[prevThree[0]]});
        prevThree[1] = prevThree[0];
        stack.pop_back();
        if (stack.size() < 2)
          break;
        prevThree[0] = *(stack.cend() - 2);
      }
    }
  }

  return out;
}

bool Triangulator::IsZeroSize(VertexID vertices[3]) const
{
  if (config.keepZeroSizeTriangle)
    return false;

  double cross = (_vertices[vertices[2]] - _vertices[vertices[1]]) ^
                 (_vertices[vertices[1]] - _vertices[vertices[0]]);
  return std::abs(cross) < config.tolerance;
}

bool Triangulator::IsConvex(VertexID vertices[3]) const
{
  double cross = (_vertices[vertices[2]] - _vertices[vertices[1]]) ^
                 (_vertices[vertices[1]] - _vertices[vertices[0]]);

  return config.useNeighborCacheToTransverse /* will be clockwise */ ? cross <= 0.
                                                                     : cross >= 0.;  // todo: or use
                                                                                     // tolerance?
}
