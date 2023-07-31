#include "triangulator.h"

#include <cassert>

Mountains Triangulator::ExtractMountains() const
{
  // todo: simplify the code

  std::vector<bool> leftSegmentVisited(_regions.Size(), false), rightSegmentVisited(_regions.Size(), false);

  Mountains mountains;

  const auto ProcedureLeft = [this, &leftSegmentVisited, &rightSegmentVisited, &mountains](
                                 const Region &region, SegmentID leftSegmentID, const Segment &leftSegment) {
    VertexID baseVertexID = leftSegment.highVertex;

    Mountain mountain;  // first and last vertex be the endpoints of the base edge

    // jump to the top then go down
    bool clockwise = false;
    if (configTri.useNeighborCacheToTransverse)
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
        if (!Valid(nextRegionID))
          break;
        curRegionPtr = &_regions[nextRegionID];
      }

      mountain.push_back(leftSegment.lowVertex);
      clockwise = true;
    }
    // go down then transverse up
    else
    {
      const Region *curRegionPtr = &region, *lastRegionPtr = curRegionPtr;

      mountain.push_back(leftSegment.lowVertex);

      // go down to the lowest region with the same left segment
      /* for this (the lowest region) region, the low vertex lies on the base segment while the top vertex
       * lies on the opposite segment (we have filter out other occasions before entering this block), hence a
       * diagonal will be drawn.
       *
       * ---@n--x
       *  Å´ |     x
       *    |-------@3            @ mountain vertices
       *  Å´ |      /
       *    |-----@2
       *  Å´ |..../.\
       *    |...+...\
       *  Å´ |../.....\  <-- go down until this region
       *    |.+.......\
       *    |/.........\
       *    @1----------\
       * */
      while (_segments[curRegionPtr->left].highVertex == baseVertexID)
      {
        lastRegionPtr = curRegionPtr;

        leftSegmentVisited[_regions.GetIndex(lastRegionPtr)] = true;

        RegionID nextRegionID = curRegionPtr->lowNeighbors[0];
        if (!Valid(nextRegionID))  // when the lowest region degenerates into a triangle
          break;

        curRegionPtr = &_regions[nextRegionID];
      }

      // go up to transverse the mountain
      /* the generated mountain will have counterclockwise vertices */
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
      clockwise = false;
    }

    assert(mountain.size() >= 3);
    if (mountain.size() >= 3)
      mountains.push_back({mountain, clockwise});
  };

  const auto ProcedureRight = [this, &leftSegmentVisited, &rightSegmentVisited, &mountains](
                                  const Region &region, SegmentID rightSegmentID,
                                  const Segment &rightSegment) {
    VertexID baseVertexID = rightSegment.highVertex;

    Mountain mountain;

    // jump to the top then go down
    bool clockwise = false;
    if (configTri.useNeighborCacheToTransverse)
    {
      // todo: check this
      mountain.push_back(baseVertexID);

      const auto &tln = _lowNeighbors[baseVertexID];  // top low neighbors
      const Region *curRegionPtr =
          _regions[tln.left].left == rightSegmentID ? &_regions[tln.left] : &_regions[tln.mid];

      while (curRegionPtr->high != rightSegment.lowVertex)
      {
        if (mountain.empty() || mountain.back() != curRegionPtr->high)
          mountain.push_back(curRegionPtr->high);

        rightSegmentVisited[_regions.GetIndex(curRegionPtr)] = true;

        RegionID nextRegionID = curRegionPtr->lowNeighbors[1];
        if (!Valid(nextRegionID))
          break;
        curRegionPtr = &_regions[nextRegionID];
      }

      mountain.push_back(rightSegment.lowVertex);
      clockwise = false;
    }
    // go down then transverse up
    else
    {
      const Region *curRegionPtr = &region, *lastRegionPtr = curRegionPtr;

      mountain.push_back(rightSegment.lowVertex);

      // go down to the lowest region with the same left segment
      while (_segments[curRegionPtr->right].highVertex == baseVertexID)
      {
        lastRegionPtr = curRegionPtr;

        rightSegmentVisited[_regions.GetIndex(lastRegionPtr)] = true;

        RegionID nextRegionID = curRegionPtr->lowNeighbors[1];
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
      clockwise = true;
    }

    assert(mountain.size() >= 3);
    if (mountain.size() >= 3)
      mountains.push_back({mountain, clockwise});
  };

  // resolve normal mountain
  for (RegionID regionID = 0, n = _regions.Size(); regionID < n; ++regionID)
  {
    const Region &region = _regions[regionID];
    if (region.depth % 2 == 0)
      continue;

    if (leftSegmentVisited[regionID] && rightSegmentVisited[regionID])  // bad
      continue;

    /* since the trapezoid is closed by the boundary, its left and right leg won't be invalid or infinite, we
     * can take the contents safely. */
    const SegmentID leftSegmentID = region.left, rightSegmentID = region.right;
    const Segment &leftSegment = _segments[leftSegmentID], rightSegment = _segments[rightSegmentID];

    // skip not diagonalized trapezoid
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

  return mountains;
}

Triangles Triangulator::TriangulateMountain(const Mountain &mountain, Triangles &out, bool clockwise) const
{
  if (configTri.mountainResolutionMethod == ConfigTri::EAR_CLIPPING)
    return EarClipping(mountain, out, clockwise);
  return ChimneyClipping(mountain, out);
}

Triangles Triangulator::Triangulate() const
{
  Triangles triangles;

  for (const auto &[mountain, cw] : ExtractMountains())
    TriangulateMountain(mountain, triangles, cw);

  return triangles;
}

Triangles Triangulator::EarClipping(const Mountain &mountain, Triangles &out, bool clockwise) const
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

  if (mountain.size() == 3)
  // degenerated mountain
  {
    out.push_back(Triangle{_vertices[mountain[0]], _vertices[mountain[1]], _vertices[mountain[2]]});
    return out;
  }

  std::vector<unsigned int> prevs, nexts, current;
  prevs.reserve(mountain.size());
  nexts.reserve(mountain.size());
  unsigned int n = static_cast<unsigned int>(mountain.size());

  for (unsigned int i = 0; i < n; ++i)
  {
    prevs.push_back(i ? i - 1 : n - 1);
    nexts.push_back(i == n - 1 ? 0 : i + 1);
  }

  for (unsigned int i = 1; i < n - 1; ++i)
  // find all convex vertices
  // first & last are vertices from the base segment, we don't consider them
  {
    if (IsConvex(mountain[prevs[i]], mountain[i], mountain[nexts[i]], clockwise))
    {
      current.push_back(i);
    }
  }
  assert(!current.empty());

  unsigned int cur = 0;
  while (n-- >= 3)
  {
    // get cut this ear
    cur = current.back();

    VertexID prev = prevs[cur], next = nexts[cur];

    Triangle triangle =
        Triangle{_vertices[mountain[prev]], _vertices[mountain[cur]], _vertices[mountain[next]]};
    if (CheckTriangle(triangle))
      out.push_back(triangle);

    if (n == 2)
      break;

    prevs[next] = prev;
    nexts[prev] = next;
    prevs[cur] = nexts[cur] = INVALID_INDEX;

    current.pop_back();

    // prev neighbor
    if (IsConvex(mountain[prevs[prev]], mountain[prev], mountain[nexts[prev]],
                 clockwise))  // not base && is convex
      current.push_back(prev);

    // next neighbor
    if (IsConvex(mountain[prevs[next]], mountain[next], mountain[nexts[next]], clockwise))
      current.push_back(next);
  }

  return out;
}

Triangles Triangulator::ChimneyClipping(const Mountain &mountain, Triangles &out) const
{
  // todo: this is highly possibly to be problematic. Re-read this.
  assert(mountain.size() > 2);
  std::vector<VertexID> stack = {mountain[0], mountain[1]};

  bool clockwise = configTri.useNeighborCacheToTransverse;

  VertexID next;
  for (size_t i = 2; i < mountain.size() && stack.size() >= 2; ++i)
  {
    next = mountain[i];

    VertexID current = stack.back(), prev = *(stack.cend() - 2);
    if (!IsConvex(next, current, prev, clockwise))
      stack.push_back(next);
    else
    {
      while (IsConvex(next, current, prev, clockwise))
      {
        Triangle triangle = Triangle{_vertices[prev], _vertices[current], _vertices[next]};
        if (CheckTriangle(triangle))
          out.push_back(triangle);
        current = next;
        stack.pop_back();
        if (stack.size() < 2)
          break;
        next = *(stack.cend() - 2);
      }
    }
  }

  return out;
}

bool Triangulator::CheckTriangle(const Triangle &triangle) const
{
  if (configTri.zeroSizeTrianglePolicy ^ ConfigTri::KEEP_LINELIKE)
  {
    if (IsZeroSize(triangle[0], triangle[1], triangle[2]))
      return false;
  }
  if (configTri.zeroSizeTrianglePolicy ^ ConfigTri::KEEP_POINTLIKE)
  {
    if (IsPointLike(triangle[0], triangle[1], triangle[2], true))
      return false;
  }
  return true;
}

bool Triangulator::IsZeroSize(VertexID prevID, VertexID currentID, VertexID nextID) const
{
  const Vertex &prev = _vertices[prevID], &current = _vertices[currentID], &next = _vertices[nextID];
  return IsZeroSize(prev, current, next);
}

bool Triangulator::IsZeroSize(const Vertex &prev, const Vertex &current, const Vertex &next) const
{
  if (configTri.zeroSizeTrianglePolicy == ConfigTri::KEEP_ALL)  // todo: handle other configurations
    return false;

  double cross = (next - current) ^ (current - prev);
  return std::abs(cross) <= config.tolerance;
}

bool Triangulator::IsPointLike(VertexID prevID,
                               VertexID currentID,
                               VertexID nextID,
                               bool assumeZeroSize) const
{
  const Vertex &prev = _vertices[prevID], &current = _vertices[currentID], &next = _vertices[nextID];
  return IsPointLike(prev, current, next, assumeZeroSize);
}

bool Triangulator::IsPointLike(const Vertex &prev,
                               const Vertex &current,
                               const Vertex &next,
                               bool assumeZeroSize /* = false */) const
{
  bool zeroSize = assumeZeroSize ? true : IsZeroSize(prev, current, next);
  if (!zeroSize)
    return false;
  return (next - current).NormSq() <= config.tolerance && (current - prev).NormSq() <= config.tolerance;
}

bool Triangulator::IsConvex(VertexID prevID, VertexID currentID, VertexID nextID, bool clockwise) const
{
  const Vertex &prev = _vertices[prevID], &current = _vertices[currentID], &next = _vertices[nextID];
  return IsConvex(prev, current, next, clockwise);
}

bool Triangulator::IsConvex(const Vertex &prev,
                            const Vertex &current,
                            const Vertex &next,
                            bool clockwise) const
{
  double cross = (next - current) ^ (current - prev);

  return clockwise ? cross >= 0. : cross <= 0.;  // todo: or use
                                                 // tolerance?
}
