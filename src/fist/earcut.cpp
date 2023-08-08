#include "earcut.h"

#include <algorithm>

Triangles EarCutTriangulator::Triangulate(const std::vector<double> &data,
                                          const std::vector<unsigned int> &holeIndices,
                                          short dim)
{
  if (dim != 2 && dim != 3)
  {
    dim = 2;
  }

  bool hasHoles         = !holeIndices.empty();
  unsigned int outerLen = hasHoles ? (holeIndices[0] * dim) : data.size();

  auto outerNode = CreateLinkedList(data, 0, outerLen, dim, true);
  Triangles triangles;

  if (!outerNode || outerNode->next == outerNode->prev)
    return triangles;

  if (hasHoles)
    outerNode == ConnectHoles(data, holeIndices, outerNode, dim);

  double minX, minY, maxX, maxY, x, y, invSize;

  // if the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox
  if (data.size() > 80 * dim)
  {
    minX = maxX = data[0];
    minY = maxY = data[1];

    for (unsigned int i = dim; i < outerLen; i += dim)
    {
      x = data[i];
      y = data[i + 1];
      if (x < minX)
        minX = x;
      if (y < minY)
        minY = y;
      if (x > maxX)
        maxX = x;
      if (y > maxY)
        maxY = y;
    }

    // minX, minY and invSize are later used to transform coordinates into integers for z-order calculation
    invSize = std::max(maxX - minX, maxY - minY);
    invSize = invSize != 0 ? 32767 / invSize : 0;
  }

  EarcutLinked(outerNode, triangles, dim, minX, minY, invSize, 0);

  return triangles;
}

Node *EarCutTriangulator::CreateLinkedList(const std::vector<double> &data,
                                           unsigned int start,
                                           unsigned int end,
                                           short dim,
                                           bool clockwise)
{
  unsigned int i;
  Node *last;

  if (clockwise == (EvalSignedArea(data, start, end, dim) > 0))
  {
    for (i = start; i < end; i += dim)
      last = InsertNode(i, data[i], data[i + 1], last);
  }
  else
  {
    for (i = end - dim; i >= start; i -= dim)
      last = InsertNode(i, data[i], data[i + 1], last);
  }

  if (last && Equals(last, last->next))
  {
    RemoveNode(last);
    last = last->next;
  }

  return last;
}

Node *EarCutTriangulator::FilterPoints(Node *start, Node *end)
{
  if (!start)
    return start;
  if (!end)
    end = start;

  Node *p = start;
  bool again;

  do
  {
    again = false;
    if (!p->steiner && (Equals(p, p->next) || EvalArea(p->prev, p, p->next) == 0))
    {
      RemoveNode(p);
      p = end = p->prev;
      if (p == p->next)
        break;
      again = true;
    }
    else
    {
      p = p->next;
    }
  } while (again || p != end);

  return end;
}

// main ear slicing loop which triangulates a polygon (given as a linked list)
void EarCutTriangulator::EarcutLinked(Node *ear,
                                      Triangles &triangles,
                                      short dim,
                                      double minX,
                                      double minY,
                                      double invSize,
                                      short pass)
{
  if (!ear)
    return;

  // interlink polygon nodes in z-order
  if (!pass && invSize)
    IndexCurve(ear, minX, minY, invSize);

  Node *stop = ear, *prev, *next;

  // iterate through ears, slicing them one by one
  while (ear->prev != ear->next)
  {
    prev = ear->prev;
    next = ear->next;

    if (invSize ? IsEarHashed(ear, minX, minY, invSize) : IsEar(ear))
    {
      // cut off the triangle
      triangles.push_back(prev->i / dim);
      triangles.push_back(ear->i / dim);
      triangles.push_back(next->i / dim);

      RemoveNode(ear);

      // skipping the next vertex leads to less sliver triangles
      ear  = next->next;
      stop = next->next;

      continue;
    }

    ear = next;

    // if we looped through the whole remaining polygon and can't find any more ears
    if (ear == stop)
    {
      // try filtering points and slicing again
      if (!pass)
      {
        EarcutLinked(FilterPoints(ear), triangles, dim, minX, minY, invSize, 1);

        // if this didn't work, try curing all small self-intersections locally
      }
      else if (pass == 1)
      {
        ear = CureLocalIntersections(FilterPoints(ear), triangles, dim);
        EarcutLinked(ear, triangles, dim, minX, minY, invSize, 2);

        // as a last resort, try splitting the remaining polygon into two
      }
      else if (pass == 2)
      {
        SplitEarcut(ear, triangles, dim, minX, minY, invSize);
      }

      break;
    }
  }
}

// check whether a polygon node forms a valid ear with adjacent nodes
bool EarCutTriangulator::IsEar(Node *ear)
{
  Node *a = ear->prev, *b = ear, *c = ear->next;

  if (EvalArea(a, b, c) >= 0)
    return false;  // reflex, can't be an ear

  // now make sure we don't have other points inside the potential ear
  double ax = a->x, bx = b->x, cx = c->x, ay = a->y, by = b->y, cy = c->y;

  // triangle bbox; min & max are calculated like this for speed
  bool x0 = ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx),
       y0 = ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy),
       x1 = ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx),
       y1 = ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy);

  Node *p = c->next;
  while (p != a)
  {
    if (p->x >= x0 && p->x <= x1 && p->y >= y0 && p->y <= y1 &&
        PointInTriangle(ax, ay, bx, by, cx, cy, p->x, p->y) && EvalArea(p->prev, p, p->next) >= 0)
      return false;
    p = p->next;
  }

  return true;
}

bool EarCutTriangulator::IsEarHashed(Node *ear, double minX, double minY, double invSize)
{
  Node *a = ear->prev, *b = ear, *c = ear->next;

  if (EvalArea(a, b, c) >= 0)
    return false;  // reflex, can't be an ear

  double ax = a->x, bx = b->x, cx = c->x, ay = a->y, by = b->y, cy = c->y;

  // triangle bbox; min & max are calculated like this for speed
  double x0 = ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx),
         y0 = ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy),
         x1 = ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx),
         y1 = ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy);

  // z-order range for the current triangle bbox;
  ZOrder minZ = GetZOrder(x0, y0, minX, minY, invSize), maxZ = GetZOrder(x1, y1, minX, minY, invSize);

  Node *p = ear->prevZ, *n = ear->nextZ;

  // look for points inside the triangle in both directions
  while (p && p->z >= minZ && n && n->z <= maxZ)
  {
    if (p->x >= x0 && p->x <= x1 && p->y >= y0 && p->y <= y1 && p != a && p != c &&
        PointInTriangle(ax, ay, bx, by, cx, cy, p->x, p->y) && EvalArea(p->prev, p, p->next) >= 0)
      return false;
    p = p->prevZ;

    if (n->x >= x0 && n->x <= x1 && n->y >= y0 && n->y <= y1 && n != a && n != c &&
        PointInTriangle(ax, ay, bx, by, cx, cy, n->x, n->y) && EvalArea(n->prev, n, n->next) >= 0)
      return false;
    n = n->nextZ;
  }

  // look for remaining points in decreasing z-order
  while (p && p->z >= minZ)
  {
    if (p->x >= x0 && p->x <= x1 && p->y >= y0 && p->y <= y1 && p != a && p != c &&
        PointInTriangle(ax, ay, bx, by, cx, cy, p->x, p->y) && EvalArea(p->prev, p, p->next) >= 0)
      return false;
    p = p->prevZ;
  }

  // look for remaining points in increasing z-order
  while (n && n->z <= maxZ)
  {
    if (n->x >= x0 && n->x <= x1 && n->y >= y0 && n->y <= y1 && n != a && n != c &&
        PointInTriangle(ax, ay, bx, by, cx, cy, n->x, n->y) && EvalArea(n->prev, n, n->next) >= 0)
      return false;
    n = n->nextZ;
  }

  return true;
}

Node *EarCutTriangulator::CureLocalIntersections(Node *start, Triangles &triangles, short dim)
{
  Node *p = start;
  do
  {
    Node *a = p->prev, *b = p->next->next;

    if (!Equals(a, b) && Intersects(a, p, p->next, b) && LocallyInside(a, b) && LocallyInside(b, a))
    {

      triangles.push_back(a->i / dim | 0);
      triangles.push_back(p->i / dim | 0);
      triangles.push_back(b->i / dim | 0);

      // remove two nodes involved
      RemoveNode(p);
      RemoveNode(p->next);

      p = start = b;
    }
    p = p->next;
  } while (p != start);

  return FilterPoints(p);
}

// try splitting polygon into two and triangulate them independently
void EarCutTriangulator::SplitEarcut(Node *start,
                                     Triangles &triangles,
                                     short dim,
                                     double minX,
                                     double minY,
                                     double invSize)
{
  // look for a valid diagonal that divides the polygon into two
  Node *a = start;
  do
  {
    Node *b = a->next->next;
    while (b != a->prev)
    {
      if (a->i != b->i && IsValidDiagonal(a, b))
      {
        // split the polygon in two by the diagonal
        Node *c = SplitPolygon(a, b);

        // filter collinear points around the cuts
        a = FilterPoints(a, a->next);
        c = FilterPoints(c, c->next);

        // run earcut on each half
        EarcutLinked(a, triangles, dim, minX, minY, invSize, 0);
        EarcutLinked(c, triangles, dim, minX, minY, invSize, 0);
        return;
      }
      b = b->next;
    }
    a = a->next;
  } while (a != start);
}

// link every hole into the outer loop, producing a single-ring polygon without holes
Node *EarCutTriangulator::ConnectHoles(const std::vector<double> &data,
                                       const std::vector<unsigned int> &holeIndices,
                                       Node *outerNode,
                                       short dim)
{
  std::vector<Node *> queue;
  queue.reserve(holeIndices.size());

  size_t i, len, start, end;
  Node *list;

  for (i = 0, len = holeIndices.size(); i < len; i++)
  {
    start = holeIndices[i] * dim;
    end   = i < len - 1 ? holeIndices[i + 1] * dim : data.size();
    list  = CreateLinkedList(data, start, end, dim, false);
    if (list == list->next)
      list->steiner = true;
    queue.push_back(GetLeftmost(list));
  }

  std::sort(queue.begin(), queue.end(), [](Node *a, Node *b) { return a->x - b->x; });

  // process holes from left to right
  for (i = 0; i < queue.size(); i++)
  {
    outerNode = ConnectHole(queue[i], outerNode);
  }

  return outerNode;
}

// find a bridge between vertices that connects hole with an outer ring and and link it
Node *EarCutTriangulator::ConnectHole(Node *hole, Node *outerNode)
{
  Node *bridge = FindHoleBridge(hole, outerNode);
  if (!bridge)
  {
    return outerNode;
  }

  Node *bridgeReverse = SplitPolygon(bridge, hole);

  // filter collinear points around the cuts
  FilterPoints(bridgeReverse, bridgeReverse->next);
  return FilterPoints(bridge, bridge->next);
}

// David Eberly's algorithm for finding a bridge between hole and outer polygon
Node *EarCutTriangulator::FindHoleBridge(Node *hole, Node *outerNode)
{
  Node *p = outerNode, *m;

  double hx = hole->x, hy = hole->y, qx = -HUGE_VAL;

  // find a segment intersected by a ray from the hole's leftmost point to the left;
  // segment's endpoint with lesser x will be potential connection point
  do
  {
    if (hy <= p->y && hy >= p->next->y && p->next->y != p->y)
    {
      double x = p->x + (hy - p->y) * (p->next->x - p->x) / (p->next->y - p->y);
      if (x <= hx && x > qx)
      {
        qx = x;
        m  = p->x < p->next->x ? p : p->next;
        if (x == hx)
          return m;  // hole touches outer segment; pick leftmost endpoint
      }
    }
    p = p->next;
  } while (p != outerNode);

  if (!m)
    return nullptr;

  // look for points inside the triangle of hole point, segment intersection and endpoint;
  // if there are no points found, we have a valid connection;
  // otherwise choose the point of the minimum angle with the ray as connection point

  Node *stop = m;
  double mx = m->x, my = m->y, tanMin = HUGE_VAL, tan;

  p = m;

  do
  {
    if (hx >= p->x && p->x >= mx && hx != p->x &&
        PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p->x, p->y))
    {

      tan = std::abs(hy - p->y) / (hx - p->x);  // tangential

      if (LocallyInside(p, hole) &&
          (tan < tanMin || (tan == tanMin && (p->x > m->x || (p->x == m->x && SectorContainsSector(m, p))))))
      {
        m      = p;
        tanMin = tan;
      }
    }

    p = p->next;
  } while (p != stop);

  return m;
}

// whether sector in vertex m contains sector in vertex p in the same coordinates
bool EarCutTriangulator::SectorContainsSector(Node *m, Node *p)
{
  return EvalArea(m->prev, m, p->prev) < 0 && EvalArea(p->next, m, m->next) < 0;
}

// interlink polygon nodes in z-order
void EarCutTriangulator::IndexCurve(Node *start, double minX, double minY, double invSize)
{
  Node *p = start;
  do
  {
    if (p->z == 0)
      p->z = GetZOrder(p->x, p->y, minX, minY, invSize);
    p->prevZ = p->prev;
    p->nextZ = p->next;
    p        = p->next;
  } while (p != start);

  p->prevZ->nextZ = nullptr;
  p->prevZ        = nullptr;

  SortLinked(p);
}

// Simon Tatham's linked list merge sort algorithm
// http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
Node *EarCutTriangulator::SortLinked(Node *list)
{
  Node *p, *q, *e, *tail;
  unsigned int i, numMerges, pSize, qSize, inSize = 1;

  do
  {
    p         = list;
    list      = nullptr;
    tail      = nullptr;
    numMerges = 0;

    while (p)
    {
      numMerges++;
      q     = p;
      pSize = 0;
      for (i = 0; i < inSize; i++)
      {
        pSize++;
        q = q->nextZ;
        if (!q)
          break;
      }
      qSize = inSize;

      while (pSize > 0 || (qSize > 0 && q))
      {

        if (pSize != 0 && (qSize == 0 || !q || p->z <= q->z))
        {
          e = p;
          p = p->nextZ;
          pSize--;
        }
        else
        {
          e = q;
          q = q->nextZ;
          qSize--;
        }

        if (tail)
          tail->nextZ = e;
        else
          list = e;

        e->prevZ = tail;
        tail     = e;
      }

      p = q;
    }

    tail->nextZ = nullptr;
    inSize *= 2;

  } while (numMerges > 1);

  return list;
}

// z-order of a point given coordinates and inverse of the longer side of data boundbox
ZOrder EarCutTriangulator::GetZOrder(double ix, double iy, double minX, double minY, double invSize)
{
  // coordinates are transformed into non-negative 15-bit integer range
  ZOrder x = (ix - minX) * invSize;
  ZOrder y = (iy - minY) * invSize;

  x = (x | (x << 8)) & 0x00FF00FF;
  x = (x | (x << 4)) & 0x0F0F0F0F;
  x = (x | (x << 2)) & 0x33333333;
  x = (x | (x << 1)) & 0x55555555;

  y = (y | (y << 8)) & 0x00FF00FF;
  y = (y | (y << 4)) & 0x0F0F0F0F;
  y = (y | (y << 2)) & 0x33333333;
  y = (y | (y << 1)) & 0x55555555;

  return x | (y << 1);
}

// find the leftmost node of a polygon ring
Node *EarCutTriangulator::GetLeftmost(Node *start)
{
  Node *p = start, *leftmost = start;
  do
  {
    if (p->x < leftmost->x || (p->x == leftmost->x && p->y < leftmost->y))
      leftmost = p;
    p = p->next;
  } while (p != start);

  return leftmost;
}

// check if a point lies within a convex triangle
bool EarCutTriangulator::PointInTriangle(double ax,
                                         double ay,
                                         double bx,
                                         double by,
                                         double cx,
                                         double cy,
                                         double px,
                                         double py)
{
  return (cx - px) * (ay - py) >= (ax - px) * (cy - py) && (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
         (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

// check if a diagonal between two polygon nodes is valid (lies in polygon interior)
bool EarCutTriangulator::IsValidDiagonal(Node *a, Node *b)
{
  return a->next->i != b->i && a->prev->i != b->i &&
         !IntersectsPolygon(a, b) &&                                           // does't intersect other edges
         (LocallyInside(a, b) && LocallyInside(b, a) && MiddleInside(a, b) &&  // locally visible
              (EvalArea(a->prev, a, b->prev) ||
               EvalArea(a, b->prev, b)) ||  // does not create opposite-facing sectors
          Equals(a, b) && EvalArea(a->prev, a, a->next) > 0 &&
              EvalArea(b->prev, b, b->next) > 0);  // special zero-length case
}

// signed area of a triangle
double EarCutTriangulator::EvalArea(Node *p, Node *q, Node *r)
{
  return (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);
}

// check if two points are equal
bool EarCutTriangulator::Equals(Node *p1, Node *p2)
{
  return p1->x == p2->x && p1->y == p2->y;
}

// check if two segments intersect
bool EarCutTriangulator::Intersects(Node *p1, Node *q1, Node *p2, Node *q2)
{
  short o1 = GetSign(EvalArea(p1, q1, p2));
  short o2 = GetSign(EvalArea(p1, q1, q2));
  short o3 = GetSign(EvalArea(p2, q2, p1));
  short o4 = GetSign(EvalArea(p2, q2, q1));

  if (o1 != o2 && o3 != o4)
    return true;  // general case

  if (o1 == 0 && OnSegment(p1, p2, q1))
    return true;  // p1, q1 and p2 are collinear and p2 lies on p1q1
  if (o2 == 0 && OnSegment(p1, q2, q1))
    return true;  // p1, q1 and q2 are collinear and q2 lies on p1q1
  if (o3 == 0 && OnSegment(p2, p1, q2))
    return true;  // p2, q2 and p1 are collinear and p1 lies on p2q2
  if (o4 == 0 && OnSegment(p2, q1, q2))
    return true;  // p2, q2 and q1 are collinear and q1 lies on p2q2

  return false;
}

// for collinear points p, q, r, check if point q lies on segment pr
bool EarCutTriangulator::OnSegment(Node *p, Node *q, Node *r)
{
  return q->x <= std::max(p->x, r->x) && q->x >= std::min(p->x, r->x) && q->y <= std::max(p->y, r->y) &&
         q->y >= std::min(p->y, r->y);
}

short EarCutTriangulator::GetSign(double num)
{
  return num > 0 ? 1 : num < 0 ? -1 : 0;
}

// check if a polygon diagonal intersects any polygon segments
bool EarCutTriangulator::IntersectsPolygon(Node *a, Node *b)
{
  Node *p = a;
  do
  {
    if (p->i != a->i && p->next->i != a->i && p->i != b->i && p->next->i != b->i &&
        Intersects(p, p->next, a, b))
      return true;
    p = p->next;
  } while (p != a);

  return false;
}

// check if a polygon diagonal is locally inside the polygon
bool EarCutTriangulator::LocallyInside(Node *a, Node *b)
{
  return EvalArea(a->prev, a, a->next) < 0 ? EvalArea(a, b, a->next) >= 0 && EvalArea(a, a->prev, b) >= 0
                                           : EvalArea(a, b, a->prev) < 0 || EvalArea(a, a->next, b) < 0;
}

// check if the middle point of a polygon diagonal is inside the polygon
bool EarCutTriangulator::MiddleInside(Node *a, Node *b)
{
  Node *p     = a;
  bool inside = false;

  double px = (a->x + b->x) / 2, py = (a->y + b->y) / 2;
  do
  {
    if (((p->y > py) != (p->next->y > py)) && p->next->y != p->y &&
        (px < (p->next->x - p->x) * (py - p->y) / (p->next->y - p->y) + p->x))
      inside = !inside;
    p = p->next;
  } while (p != a);

  return inside;
}

// link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits polygon into
// two; if one belongs to the outer ring and another to a hole, it merges it into a single ring
Node *EarCutTriangulator::SplitPolygon(Node *a, Node *b)
{
  Node *a2 = new Node(a->i, a->x, a->y), *b2 = new Node(b->i, b->x, b->y), *an = a->next, *bp = b->prev;

  a->next = b;
  b->prev = a;

  a2->next = an;
  an->prev = a2;

  b2->next = a2;
  a2->prev = b2;

  bp->next = b2;
  b2->prev = bp;

  return b2;
}

// create a node and optionally link it with previous one (in a circular doubly linked list)
Node *EarCutTriangulator::InsertNode(NodeID i, double x, double y, Node *last)
{
  Node *p = new Node(i, x, y);

  if (!last)
  {
    p->prev = p;
    p->next = p;
  }
  else
  {
    p->next          = last->next;
    p->prev          = last;
    last->next->prev = p;
    last->next       = p;
  }
  return p;
}

void EarCutTriangulator::RemoveNode(Node *p)
{
  p->next->prev = p->prev;
  p->prev->next = p->next;

  if (p->prevZ)
    p->prevZ->nextZ = p->nextZ;
  if (p->nextZ)
    p->nextZ->prevZ = p->prevZ;
}

//// return a percentage difference between the polygon area and its triangulation area;
//// used to verify correctness of triangulation
// bool Deviation(const std::vector<double>& data, const std::vector<unsigned int>& holeIndices, short dim,
// Triangles& triangles)
//{
//   var hasHoles = holeIndices && holeIndices.length;
//   var outerLen = hasHoles ? holeIndices[0] * dim : data.length;
//
//   var polygonArea = Math.abs(signedArea(data, 0, outerLen, dim));
//   if (hasHoles)
//   {
//     for (var i = 0, len = holeIndices.length; i < len; i++)
//     {
//       var start = holeIndices[i] * dim;
//       var end   = i < len - 1 ? holeIndices[i + 1] * dim : data.length;
//       polygonArea -= Math.abs(signedArea(data, start, end, dim));
//     }
//   }
//
//   var trianglesArea = 0;
//   for (i = 0; i < triangles.length; i += 3)
//   {
//     var a = triangles[i] * dim;
//     var b = triangles[i + 1] * dim;
//     var c = triangles[i + 2] * dim;
//     trianglesArea += Math.abs((data[a] - data[c]) * (data[b + 1] - data[a + 1]) -
//                               (data[a] - data[b]) * (data[c + 1] - data[a + 1]));
//   }
//
//   return polygonArea == 0 && trianglesArea == 0 ? 0 : Math.abs((trianglesArea - polygonArea) /
//   polygonArea);
// };

double EarCutTriangulator::EvalSignedArea(const std::vector<double> &data,
                                          unsigned int start,
                                          unsigned int end,
                                          short dim)
{
  unsigned int sum = 0;
  for (unsigned int i = start, j = end - dim; i < end; i += dim)
  {
    sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
    j = i;
  }
  return sum;
}

//// turn a polygon in a multi-dimensional array form (e.g. as in GeoJSON) into a form Earcut accepts
// earcut.flatten = function(data)
//{
//   var dim = data[0][0].length, result = {vertices : [], holes : [], dimensions : dim}, holeIndex = 0;
//
//   for (var i = 0; i < data.length; i++)
//   {
//     for (var j = 0; j < data[i].length; j++)
//     {
//       for (var d = 0; d < dim; d++)
//         result.vertices.push(data[i][j][d]);
//     }
//     if (i > 0)
//     {
//       holeIndex += data[i - 1].length;
//       result.holes.push(holeIndex);
//     }
//   }
//   return result;
// };