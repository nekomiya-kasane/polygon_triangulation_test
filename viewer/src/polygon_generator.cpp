#include "polygon_generator.h"

#include <algorithm>
#include <random>
#include <set>
#include <unordered_map>
#include <cassert>
#include <limits>

using IndexPair  = std::pair<int, int>;
using IndexPairs = std::vector<IndexPair>;

auto RecombineEdges(const std::set<IndexPair>& edges)
{
  std::unordered_map<int, std::set<int>> dict;

  for (auto [a, b] : edges)
  {
    assert(a != b);
    dict[a].insert(b);
    dict[b].insert(a);
  }

  std::vector<int> polygon;
  
  int firstIndex = std::numeric_limits<int>::max(), lastIndex;
  for (const auto& [k, _] : dict)
  {
    if (k < firstIndex)
      firstIndex = k;
  }

  polygon.push_back(firstIndex);
  
  lastIndex = polygon.back();
  while (true)
  {
    auto nextIndex = *dict[lastIndex].rbegin();
    dict[lastIndex].erase(nextIndex);

    dict[nextIndex].erase(lastIndex);

    if (nextIndex == firstIndex)
      break;
    polygon.push_back(nextIndex);
    lastIndex = nextIndex;
  }
  
  return polygon;
}

double Orientation(const Vec2& u, const Vec2& v, const Vec2& w)
{
  return (u - w) ^ (v - w);
}

bool ProjectionsIntersect(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d)
{
  if (std::max(a.x, b.x) < std::min(c.x, d.x))
    return false;
  if (std::max(c.x, d.x) < std::min(a.x, b.x))
    return false;
  if (std::max(a.y, b.y) < std::min(c.y, d.y))
    return false;
  if (std::max(c.y, d.y) < std::min(a.y, b.y))
    return false;
  return true;
}

bool SegmentCross(const Vec2 &a, const Vec2 &b, const Vec2 &c, const Vec2 &d)
{
  auto eps = 1e-10;
  if (std::abs(Orientation(a, b, c)) < eps && std::abs(Orientation(a, b, d)) < eps)
    return ProjectionsIntersect(a, b, c, d);
  if (Orientation(a, b, c) * Orientation(a, b, d) > 0.)
    return false;
  if (Orientation(c, d, a) * Orientation(c, d, b) > 0.)
    return false;
  return true;
}

IndexPair Ordered(int a, int b)
{
  return a > b ? IndexPair{b, a} : IndexPair{a, b};
}

IndexPairs FindIntersectingEdges(const IndexPair &edge,
                                 const std::set<IndexPair> &edges,
                                 const Vec2Set &points,
                                 bool onlyFirst)
{
  IndexPairs result;
  auto [a, b] = edge;
  Vec2 pa = points[a], pb = points[b];
  for (auto [c, d] : edges)
  {
    if (a == c || a == d || b == c || b == d)
      continue;
    Vec2 pc = points[c], pd = points[d];
    if (SegmentCross(pa, pb, pc, pd))
    {
      result.push_back({c, d});
      if (onlyFirst)
        break;
    }
  }
  return result;
}

bool EdgesConnected(const std::set<IndexPair>& edges) {
  std::unordered_map<int, std::set<int>> dict;

  for (auto [a, b] : edges)
  {
    assert(a != b);
    dict[a].insert(b);
    dict[b].insert(a);
  }

  std::set<int> verticesToVisit, verticesVisited;
  for (auto& [k, v] : dict)
  {
    if (v.size() != 2)
      return false;
    verticesToVisit.insert(k);
  }

  auto v = (*dict.begin()).first;
  while (true)
  {
    verticesVisited.insert(v);
    auto a = *dict[v].rbegin();
    dict[v].erase(a);
    auto b = *dict[v].rbegin();
    dict[v].erase(b);
    if (verticesVisited.contains(a) && verticesVisited.contains(b))
      break;
    if (!verticesVisited.contains(a))
      v = a;
    if (!verticesVisited.contains(b))
      v = b;
  }

  return verticesToVisit == verticesVisited;
}

Vec2Set PolygonGenerator::GenerateRandomPolygon(size_t num,
                                                int xmin,
                                                int xmax,
                                                int ymin,
                                                int ymax)
{
  Vec2Set points = GenerateRandomUniquePoints(num, xmin, xmax, ymin, ymax);

  std::set<IndexPair> edgesToCheck, nonIntersectingEdges;
  for (int i = 0; i < num; ++i)
    edgesToCheck.insert(Ordered(i, (i + 1) % static_cast<int>(num)));

  while (!edgesToCheck.empty())
  {
    auto [a, b] = *edgesToCheck.rbegin();
    edgesToCheck.erase(IndexPair{a, b});

    auto intersectingEdges = FindIntersectingEdges(IndexPair{a, b}, edgesToCheck, points, true);

    if (intersectingEdges.empty())
    {
      nonIntersectingEdges.insert(Ordered(a, b));
      continue;
    }

    auto [c, d] = *intersectingEdges.cbegin();
    edgesToCheck.erase(IndexPair{c, d});

    std::set<IndexPair> graph;
    std::set_union(edgesToCheck.begin(), edgesToCheck.end(), intersectingEdges.begin(),
                   intersectingEdges.end(), std::inserter(graph, graph.begin()));
    graph.insert(Ordered(c, a));
    graph.insert(Ordered(d, b));
    IndexPair newEdges[2];
    if (EdgesConnected(graph))
    {
      newEdges[0] = Ordered(c, a);
      newEdges[1] = Ordered(d, b);
    }
    else
    {
      newEdges[0] = Ordered(c, b);
      newEdges[1] = Ordered(d, a);
    }

    for (int i = 0; i < 2; ++i)
    {
      edgesToCheck.insert(newEdges[i]);
      auto intersectingEdges = FindIntersectingEdges(newEdges[i], nonIntersectingEdges, points, false);
      for (auto &intersectingEdge : intersectingEdges)
      {
        if (nonIntersectingEdges.contains(intersectingEdge))
        {
          edgesToCheck.insert(intersectingEdge);
          nonIntersectingEdges.erase(intersectingEdge);
        }
      }
    }
  }

  auto vertices = RecombineEdges(nonIntersectingEdges);

  Vec2Set result;
  result.reserve(vertices.size());
  for (auto vertexID : vertices)
    result.push_back(points[vertexID]);

  return result;
}

Vec2Set PolygonGenerator::GenerateRandomUniquePoints(size_t num,
                                                     int xmin /* = 0 */,
                                                     int xmax /* = 1000 */,
                                                     int ymin /* = 0 */,
                                                     int ymax /* = 1000 */)
{
  std::random_device rd;
  std::mt19937 generator(rd());

  std::uniform_int_distribution<int> distX(xmin, xmax);
  std::uniform_int_distribution<int> distY(ymin, ymax);

  std::set<Vec2> uniquePoints;

  while (uniquePoints.size() < num)
    uniquePoints.insert({static_cast<double>(distX(generator)), static_cast<double>(distY(generator))});

  Vec2Set points;
  for (const auto &point : uniquePoints)
    points.push_back(point);

  return points;
}