#include "viewable.h"

#include "easyx.h"
#include "graphics.h"

#include <iterator>
#include <limits>
#include <string>

ViewableTriangulator::~ViewableTriangulator()
{
  Triangulator::~Triangulator();

  delete methods.vertexDrawer;
  delete methods.regionDrawer;
  delete methods.segmentDrawer;
  delete methods.triangleDrawer;
  delete methods.mountainDrawer;
}

void ViewableTriangulator::SetOrigin(Vec2 origin)
{
  _origin = origin;
}

void ViewableTriangulator::Draw(Vec2 centroid, Vec2 factor)
{
  if (!_needRedraw)
    return;

  _centroid = centroid;
  _factor   = factor;

  if (!methods.vertexDrawer)
  {
    methods.vertexDrawer = new VertexDrawer([this](const Vertex &vertex, const std::string &label) {
      // draw horizontal line
      LINESTYLE oldStyle;
      getlinestyle(&oldStyle);

      setlinecolor(YELLOW);
      setlinestyle(PS_DASH | PS_ENDCAP_ROUND);

      VertexID id = _vertices.GetIndex(&vertex);
      POINT pt[2] = {{0, y(vertex.y)}, {1024, y(vertex.y)}};

      const Region &leftRegion = _regions[_lowNeighbors[id].left];
      if (!Infinite(leftRegion.left) && Valid(leftRegion.left))
      {
        pt[0].x = evalX(vertex.y, _segments[leftRegion.left]);
      }
      const Region &rightRegion = Valid(_lowNeighbors[id].right) ? _regions[_lowNeighbors[id].right]
                                                                 : _regions[_lowNeighbors[id].left];
      if (!Infinite(rightRegion.right) && Valid(leftRegion.right))
      {
        pt[1].x = evalX(vertex.y, _segments[rightRegion.right]);
      }
      line(pt[0].x, pt[0].y, pt[1].x, pt[1].y);

      setlinestyle(&oldStyle);

      // draw vertex
      setlinecolor(WHITE);
      circle((int)x(vertex.x), (int)y(vertex.y), (int)drawingConfig.vertexRadius);
      setfillcolor(LIGHTRED);
      settextcolor(LIGHTRED);
      fillcircle((int)x(vertex.x), (int)y(vertex.y), (int)drawingConfig.vertexRadius);
      outtextxy((int)x(vertex.x) + drawingConfig.vertexRadius, (int)y(vertex.y) + drawingConfig.vertexRadius,
                label.c_str());
    });

    methods.segmentDrawer = new SegmentDrawer([this](const Segment &segment, const std::string &) {
      const Vertex &highVertex = _vertices[segment.highVertex];
      const Vertex &lowVertex  = _vertices[segment.lowVertex];
      setlinecolor(LIGHTCYAN);
      line((int)x(highVertex.x), (int)y(highVertex.y), (int)x(lowVertex.x), (int)y(lowVertex.y));
    });

    methods.regionDrawer = new RegionDrawer([this](const Region &region, const std::string &label) {
      LINESTYLE oldStyle;
      getlinestyle(&oldStyle);

      if (region.depth != 0)
      {
        auto color = region.depth % 2 == 1 ? RGB(77, 77, 0) : RGB(0, 77, 77);
        setlinecolor(RGB(128, 128, 128));
        setlinestyle(PS_NULL);
        setfillcolor(color);
        const Segment &left = _segments[region.left], &right = _segments[region.right];
        const Vec2 ll = _vertices[left.lowVertex], lh = _vertices[left.highVertex],
                   rl = _vertices[right.lowVertex], rh = _vertices[right.highVertex];
        double highY = _vertices[region.high].y, lowY = _vertices[region.low].y;
        if (highY == lowY)
          return;

        POINT pts[4] = {{evalX(lowY, left), y(lowY)},
                        {evalX(lowY, right), y(lowY)},
                        {evalX(highY, right), y(highY)},
                        {evalX(highY, left), y(highY)}};

        // degenerated
        if (left.lowVertex == right.lowVertex)
          fillpolygon((POINT *)(pts + 1), 3);
        else if (left.highVertex == right.highVertex)
          fillpolygon((POINT *)pts, 3);
        // non-degenerated
        else
          fillpolygon(pts, 4);

        int midX = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;

        settextcolor(color * 3);
        outtextxy(midX, (pts[0].y + pts[2].y) / 2, label.c_str());
        setlinestyle(&oldStyle);
      }
    });

    methods.triangleDrawer = new TriangleDrawer([this](const Triangle &tri, const std::string &) {
      POINT pts[3];
      pts[0].x = x(tri[0].x);
      pts[0].y = y(tri[0].y);
      pts[1].x = x(tri[1].x);
      pts[1].y = y(tri[1].y);
      pts[2].x = x(tri[2].x);
      pts[2].y = y(tri[2].y);

      setlinecolor(RGB(128, 255, 0));
      polygon(pts, 3);
    });

    methods.mountainDrawer = new MountainDrawer([this](const Mountain &mountain, const std::string &) {
      POINT *pts = new POINT[mountain.size() + 1];
      for (int i = 0; i < mountain.size(); ++i)
      {
        const Vertex &vert = _vertices[mountain[i]];
        pts[i].x           = x(vert.x);
        pts[i].y           = y(vert.y);
      }
      pts[mountain.size()].x = x(_vertices[mountain[0]].x);
      pts[mountain.size()].y = y(_vertices[mountain[0]].y);
      delete[] pts;
    });
  }

  size_t i = 0;
  if (methods.mountainDrawer)
    for (const auto &mountain : _mountains)
      (*methods.mountainDrawer)(mountain.first, "M" + std::to_string(i++));

  i = 0;
  if (methods.regionDrawer)
    for (const auto &region : _regions)
      (*methods.regionDrawer)(region, "S" + std::to_string(i++));

  i = 0;
  if (methods.triangleDrawer)
    for (const auto &triangle : _triangles)
      (*methods.triangleDrawer)(triangle, "T" + std::to_string(i++));

  i = 0;
  if (methods.segmentDrawer)
    for (const auto &segment : _segments)
      (*methods.segmentDrawer)(segment, "e" + std::to_string(i++));

  i = 0;
  if (methods.vertexDrawer)
    for (const auto &vertex : _vertices)
      (*methods.vertexDrawer)(vertex, "v" + std::to_string(i++));
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
  const_cast<Triangles &>(_triangles) = Triangulator::Triangulate();  // hack it
  return _triangles;
}

Mountains ViewableTriangulator::ExtractMountains() const
{
  Mountains res = Triangulator::ExtractMountains();
  // std::copy(res.begin(), res.end(), std::back_inserter(_mountains));
  return res;
}

int ViewableTriangulator::evalX(double iy, const Segment &seg) const
{
  const Vec2 low = _vertices[seg.lowVertex], high = _vertices[seg.highVertex];
  if (low.y == high.y)
    return static_cast<int>((low.x + high.x) / 2);
  return x((iy - low.y) / (high.y - low.y) * (high.x - low.x) + low.x);
}
