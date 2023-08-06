#include "viewable.h"

#ifdef USE_EASYX
#  include "easyx.h"
#  include "graphics.h"
#else
#  pragma warning(push, 0)
#  include "raylib.h"
#  include "raymath.h"
#  include "rlgl.h"
#  pragma warning(pop)
#endif

#include <iterator>
#include <limits>
#include <sstream>
#include <string>

ViewableTriangulator::~ViewableTriangulator()
{
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

void ViewableTriangulator::SetBox(Vec2 box)
{
  _box = box;
}

void ViewableTriangulator::SetFocus(Vec2 focus)
{
  _focus = focus;
}

void ViewableTriangulator::SetZoomFactor(float factor)
{
  _zoom = factor;
}

#pragma warning(push)
#pragma warning(disable : 4244)
void ViewableTriangulator::Draw(Vec2 centroid, Vec2 factor)
{
  if (!_needRedraw)
    return;

  _centroid = centroid;
  _factor   = factor;

#ifdef USE_EASYX
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
#else
  if (!methods.vertexDrawer)
  {
    methods.vertexDrawer = new VertexDrawer([this](const Vertex &vertex, const std::string &label,
                                                   Color color, bool overrideColor) {
      // draw horizontal line LINESTYLE oldStyle;
      VertexID id   = _vertices.GetIndex(&vertex);
      Vector2 pt[2] = {{0, y(vertex.y)}, {8192, y(vertex.y)}};

      const Region &leftRegion = _regions[_lowNeighbors[id].left];
      if (!Infinite(leftRegion.left) && Valid(leftRegion.left))
      {
        pt[0].x = evalX(y(vertex.y), _segments[leftRegion.left]);
      }
      const Region &rightRegion = Valid(_lowNeighbors[id].right) ? _regions[_lowNeighbors[id].right]
                                                                 : _regions[_lowNeighbors[id].left];
      if (!Infinite(rightRegion.right) && Valid(leftRegion.right))
      {
        pt[1].x = evalX(y(vertex.y), _segments[rightRegion.right]);
      }
      DrawLineEx(pt[0], pt[1], 1.f / _zoom, Fade(YELLOW, 0.35f));

      // indicator
      if (std::abs(_focus.x - x(vertex.x)) <= drawingConfig.vertexRadius &&
          std::abs(_focus.y - y(vertex.y)) <= drawingConfig.vertexRadius)
        indicators.curVertexID = _vertices.GetIndex(&vertex);

      // draw points
      DrawCircle(x(vertex.x), y(vertex.y), drawingConfig.vertexRadius / _zoom, overrideColor ? color : RED);
      DrawCircleLines(x(vertex.x), y(vertex.y), drawingConfig.vertexRadius / _zoom, WHITE);
      DrawTextEx(drawingConfig.font, label.c_str(),
                 Vector2{x(vertex.x) + drawingConfig.vertexRadius, y(vertex.y) + drawingConfig.vertexRadius},
                 20.f / _zoom, 0.f, overrideColor ? color : RED);
    });
  }

  if (!methods.segmentDrawer)
  {
    methods.segmentDrawer = new SegmentDrawer(
        [this](const Segment &segment, const std::string &label, Color color, bool overrideColor) {
          const Vertex &highVertex = _vertices[segment.highVertex];
          const Vertex &lowVertex  = _vertices[segment.lowVertex];
          auto midX = (highVertex.x + lowVertex.x) / 2.f, midY = (highVertex.y + lowVertex.y) / 2.f;
          DrawLineEx(Vector2{x(highVertex.x), y(highVertex.y)}, Vector2{x(lowVertex.x), y(lowVertex.y)},
                     2 / _zoom, overrideColor ? color : BLUE);
          DrawTextEx(drawingConfig.font, label.c_str(), Vector2{x(midX), y(midY)}, 20.f / _zoom, 0.f,
                     overrideColor ? color : BLUE);
        });
  }

  if (!methods.regionDrawer)
  {
    methods.regionDrawer = new RegionDrawer([this](const Region &region, const std::string &label,
                                                   Color color, bool overrideColor) {
      // caution: here highY is projected to the screen coordinate system, hence numerically, highY <=
      // lowY
      float midX = 0, midY = 0;
      double highY = 0, lowY = 0, lowHighX = 0, lowLowX = 0, highLowX = 0, highHighX = 0;

      highY = !Infinite(region.high) ? y(_vertices[region.high].y) : 0;
      lowY  = !Infinite(region.low) ? y(_vertices[region.low].y) : _box.y - 5;
      if (!Infinite(region.left))
      {
        const Segment &left = _segments[region.left];
        const Vec2 ll = _vertices[left.lowVertex], lh = _vertices[left.highVertex];
        lowHighX  = evalX(lowY, left);
        highHighX = evalX(highY, left);
      }
      else
      {
        lowHighX = highHighX = 0;
      }
      if (!Infinite(region.right))
      {
        const Segment &right = _segments[region.right];
        const Vec2 rl = _vertices[right.lowVertex], rh = _vertices[right.highVertex];
        lowLowX  = evalX(lowY, right);
        highLowX = evalX(highY, right);
      }
      else
      {
        lowLowX = highLowX = _box.x - 5;
      }

      Vector2 pts[4] = {{lowHighX, lowY}, {lowLowX, lowY}, {highLowX, highY}, {highHighX, highY}};
      midX           = (lowHighX + lowLowX + highHighX + highLowX) / 4;
      midY           = (highY + lowY) / 2;

      assert(lowHighX >= 0 && lowHighX <= _box.x);
      assert(lowLowX >= 0 && lowLowX <= _box.x);
      assert(highLowX >= 0 && highLowX <= _box.x);
      assert(highHighX >= 0 && highHighX <= _box.x);

      assert(lowY >= 0 && lowY <= _box.y);
      assert(highY >= 0 && highY <= _box.y);

      assert(midX >= 0 && midX <= _box.x);
      assert(midY >= 0 && midY <= _box.y);

      // highlight regions containing the cursor
      color = overrideColor ? color
                            : ((region.depth != INVALID_DEPTH && (region.depth % 2 == 1)) ? YELLOW : GRAY);
      if (_focus.y <= (double)(int)lowY && _focus.y >= (double)(int)highY)
      {
        auto left  = evalX(_focus.y, {lowHighX, lowY}, {highHighX, highY}),
             right = evalX(_focus.y, {lowLowX, lowY}, {highLowX, highY});
        // assert(right - left > -10);  // strange
        if (_focus.x >= left && _focus.x <= right)
          // color = PURPLE;
          indicators.curRegionID = _regions.GetIndex(&region);
      }

      // background
      DrawTriangle(pts[0], pts[1], pts[2], Fade(color, 0.3f));
      DrawTriangle(pts[0], pts[2], pts[3], Fade(color, 0.3f));

      DrawTextEx(drawingConfig.font, label.c_str(), Vector2{midX, midY}, 20.f / _zoom, 0.f, color);
      DrawCircle(static_cast<int>(midX), static_cast<int>(midY), 2 / _zoom, color);
    });
  }

  if (!methods.triangleDrawer)
  {
    methods.triangleDrawer = new TriangleDrawer(
        [this](const Triangle &tri, const std::string &label, Color color, bool overrideColor) {
          Vector2 pts[3];
          pts[0].x = x(tri[0].x);
          pts[0].y = y(tri[0].y);
          pts[1].x = x(tri[1].x);
          pts[1].y = y(tri[1].y);
          pts[2].x = x(tri[2].x);
          pts[2].y = y(tri[2].y);

          float midX = (pts[0].x + pts[1].x + pts[2].x) / 3, midY = (pts[0].y + pts[1].y + pts[2].y) / 3;

          DrawTriangleLines(pts[0], pts[1], pts[2], overrideColor ? color : GREEN);
          DrawTextEx(drawingConfig.font, label.c_str(), Vector2{midX, midY}, 20 / _zoom, 0,
                     overrideColor ? color : GREEN);
        });
  }

  if (!methods.mountainDrawer)
  {
    methods.mountainDrawer = new MountainDrawer(
        [this](const Mountain &mountain, const std::string &label, Color color, bool overrideColor) {
          for (Mountain::size_type i = 0; i < mountain.size() - 1; ++i)
            DrawLineEx(Vector2{x(_vertices[mountain[i]].x), y(_vertices[mountain[i]].y)},
                       Vector2{x(_vertices[mountain[i + 1]].x), y(_vertices[mountain[i + 1]].y)}, 2 / _zoom,
                       Fade(overrideColor ? color : PURPLE, 0.3f));
          DrawLineEx(Vector2{x(_vertices[mountain.back()].x), y(_vertices[mountain.back()].y)},
                     Vector2{x(_vertices[mountain[0]].x), y(_vertices[mountain[0]].y)}, 2 / _zoom,
                     Fade(overrideColor ? color : PURPLE, 0.3f));
        });
  }
#endif

  indicators.curRegionID = indicators.curSegmentID = indicators.curVertexID = INVALID_INDEX;

  // draw shapes
  size_t i = 0;
  if (methods.mountainDrawer)
    for (const auto &mountain : _mountains)
      (*methods.mountainDrawer)(mountain.first, "M" + std::to_string(i++), WHITE, false);

  i = 0;
  if (methods.regionDrawer)
    for (const auto &region : _regions)
      (*methods.regionDrawer)(region, "S" + std::to_string(i++), WHITE, false);

  i = 0;
  if (methods.triangleDrawer)
    for (const auto &triangle : _triangles)
      (*methods.triangleDrawer)(triangle, "T" + std::to_string(i++), WHITE, false);

  i = 0;
  if (methods.segmentDrawer)
  {
    for (const auto &segmentID : _permutation)
      (*methods.segmentDrawer)(_segments[segmentID], "e" + std::to_string(segmentID), WHITE, false);
    for (SegmentID i = _segmentCount; i < _segments.Size(); ++i)
      (*methods.segmentDrawer)(_segments[i], "e" + std::to_string(i), WHITE, false);
  }

  i = 0;
  if (methods.vertexDrawer)
    for (const auto &vertex : _vertices)
      (*methods.vertexDrawer)(vertex, "v" + std::to_string(i++), WHITE, false);

  std::stringstream ss;
  ss << "D: " << config.seed << " V: " << _vertices.Size() << " T: " << _triangles.size()
     << " R: " << _regions.Size() << "/" << _regions.Capability() << " N: " << _nodes.Size() << "/"
     << _nodes.Capability() << std::endl;
  _infoBuf += ss.str();

  if (Valid(indicators.curRegionID) && !Infinite(indicators.curRegionID))
  {
    Region &region = _regions[indicators.curRegionID];
    if (methods.regionDrawer)
    {
      for (const auto neighborID :
           {region.highNeighbors[0], region.highNeighbors[1], region.lowNeighbors[0], region.lowNeighbors[1]})
        if (Valid(neighborID) && !Infinite(neighborID))
          (*methods.regionDrawer)(_regions[neighborID], "S" + std::to_string(neighborID), PURPLE, true);
      (*methods.regionDrawer)(region, "S" + std::to_string(indicators.curRegionID), Color(255, 145, 40),
                              true);
    }
    if (methods.segmentDrawer)
    {
      if (!Infinite(region.left))
        (*methods.segmentDrawer)(_segments[region.left], "e" + std::to_string(region.left), WHITE, true);
      if (!Infinite(region.right))
        (*methods.segmentDrawer)(_segments[region.right], "e" + std::to_string(region.right), WHITE, true);
    }
    if (methods.vertexDrawer)
    {
      if (!Infinite(region.low) && Valid(region.low))
        (*methods.vertexDrawer)(_vertices[region.low], "v" + std::to_string(region.low), WHITE, true);
      if (!Infinite(region.high) && Valid(region.low))
        (*methods.vertexDrawer)(_vertices[region.high], "v" + std::to_string(region.high), WHITE, true);
    }

    _infoBuf += "S" + std::to_string(indicators.curRegionID) + ": " + region.ToString() + "\n";
  }

  if (Valid(indicators.curVertexID) && !Infinite(indicators.curVertexID) && methods.vertexDrawer)
  {
    const Vertex &vertex = _vertices[indicators.curVertexID];
    (*methods.vertexDrawer)(vertex, "v" + std::to_string(indicators.curVertexID), YELLOW, true);

    _infoBuf += "V" + std::to_string(indicators.curVertexID) + ": (" + std::to_string(vertex.x) + ", " +
                std::to_string(vertex.y) + ")\n";
  }
}
#pragma warning(pop)

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

  // for generating test cases
#ifdef _DEBUG
  if (config.printCase)
  {
    size_t i = 0;
    std::cout << "\n// check the trapezoid map" << std::endl;
    auto idStr = [](AnyID id) -> std::string {
      if (!Valid(id))
        return "Nil";
      if (Infinite(id))
        return "Inf";
      return std::to_string(id);
    };

    for (const auto &node : _nodes)
    {
      std::cout << "const auto &node" << i << " = triangulator._nodes[" << i << "];\n";
      std::cout << "EXPECT_NODE(node" << i << ", " << i << ", Node::"
                << ((node.type == Node::REGION)    ? "REGION"
                    : (node.type == Node::SEGMENT) ? "SEGMENT"
                                                   : "VERTEX")
                << ", " << idStr(node.value) << ", " << idStr(node.left) << ", " << idStr(node.right)
                << ");\n";
      ++i;
    }
    i = 0;
    std::cout << "\n// check regions " << std::endl;
    for (const auto &region : _regions)
    {
      std::cout << "const auto &s" << i << " = triangulator._regions[" << i << "];\n";
      std::cout << "EXPECT_REGION(s" << i << ", " << region.nodeID << ", " << idStr(region.high) << ", "
                << idStr(region.low) << ", " << idStr(region.left) << ", " << idStr(region.right) << ", "
                << idStr(region.lowNeighbors[0]) << ", " << idStr(region.lowNeighbors[1]) << ", "
                << idStr(region.highNeighbors[0]) << ", " << idStr(region.highNeighbors[1]) << ", "
                << region.depth << ");\n";
      ++i;
    }
    i = 0;
    std::cout << "\n// check low neighbors of vertices " << std::endl;
    for (const auto &lowNeis : _lowNeighbors)
    {
      std::cout << "const auto &n" << i << " = triangulator._lowNeighbors[" << i << "];\n";
      std::cout << "EXPECT_LOW_NEIGHBOR(n" << i << ", " << idStr(lowNeis.left) << ", " << idStr(lowNeis.mid)
                << ", " << idStr(lowNeis.right) << ");\n";
      ++i;
    }
    std::cout << std::endl;
  }
#endif

  return _triangles;
}

Mountains ViewableTriangulator::ExtractMountains() const
{
  const_cast<Mountains &>(_mountains) = Triangulator::ExtractMountains();
  return _mountains;
}

#ifdef USE_EASYX
int ViewableTriangulator::evalX(double iy, const Segment &seg) const
{
  const Vec2 low = _vertices[seg.lowVertex], high = _vertices[seg.highVertex];
  if (low.y == high.y)
    return static_cast<int>((low.x + high.x) / 2);
  return x((iy - low.y) / (high.y - low.y) * (high.x - low.x) + low.x);
}
#else
float ViewableTriangulator::evalX(double iy, const Segment &seg) const
{
  const Vec2 low = _vertices[seg.lowVertex], high = _vertices[seg.highVertex];
  return evalX(iy, {x(low.x), y(low.y)}, {x(high.x), y(high.y)});
}

float ViewableTriangulator::evalX(double iy, const Vec2 &low, const Vec2 &high) const
{
  if (low.y == high.y)
    return static_cast<float>((low.x + high.x) / 2);
  return static_cast<float>((iy - low.y) / (high.y - low.y) * (high.x - low.x) + low.x);
}
#endif
