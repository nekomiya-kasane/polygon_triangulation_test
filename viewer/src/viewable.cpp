#include "viewable.h"

#include "easyx.h"
#include "graphics.h"

#include <iterator>
#include <limits>

void ViewableTriangulator::Draw(Vec2 origin, Vec2 factor)
{
  if (!_needRedraw)
    return;

  if (!methods.vertexDrawer)
  {
    methods.vertexDrawer  = new VertexDrawer([this](const Vertex &vertex) {
      circle((int)vertex.x, (int)vertex.y, (int)drawingConfig.vertexRadius);
    });
    methods.segmentDrawer = new SegmentDrawer([this](const Segment &segment) {
      const Vertex &highVertex = _vertices[segment.highVertex];
      const Vertex &lowVertex  = _vertices[segment.lowVertex];
      line((int)highVertex.x, (int)highVertex.y, (int)lowVertex.x, (int)lowVertex.y);
    });
  }

  for (const auto &vertex : _vertices)
    if (methods.vertexDrawer)
      (*methods.vertexDrawer)(vertex);

  for (const auto &segment : _segments)
    if (methods.segmentDrawer)
      (*methods.segmentDrawer)(segment);

  for (const auto &region : _regions)
    if (methods.regionDrawer)
      (*methods.regionDrawer)(region);

  for (const auto &mountain : _mountains)
    if (methods.mountainDrawer)
      (*methods.mountainDrawer)(mountain);

  for (const auto &triangle : _triangles)
    if (methods.triangleDrawer)
      (*methods.triangleDrawer)(triangle);
}

void ViewableTriangulator::GetBoundingBox(Vec2 &leftTop, Vec2 &rightBottom) const
{
  const double big = HUGE_VAL;
  leftTop.x        = big;
  leftTop.y        = -big;
  rightBottom.x    = -big;
  rightBottom.y    = big;

  for (const auto &vertex : _vertices)
  {
    if (vertex.x > rightBottom.x)
      rightBottom.x = vertex.x;
    if (vertex.x < leftTop.x)
      leftTop.x = vertex.x;
    if (vertex.y > leftTop.y)
      leftTop.y = vertex.y;
    if (vertex.y < rightBottom.y)
      rightBottom.y = vertex.y;
  }
}

Triangles ViewableTriangulator::Triangulate() const
{
  Triangles res = Triangulator::Triangulate();
  // std::copy(res.begin(), res.end(), std::back_inserter(_triangles));
  return res;
}

Mountains ViewableTriangulator::ExtractMountains() const
{
  Mountains res = Triangulator::ExtractMountains();
  // std::copy(res.begin(), res.end(), std::back_inserter(_mountains));
  return res;
}
