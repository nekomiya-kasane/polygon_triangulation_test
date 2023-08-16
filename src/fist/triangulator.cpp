#include "triangulator.h"

#include <functional>
#include <limits>

//=======================================================
// Node
//=======================================================

constexpr const char *const nodeFiller()
{
  char *const result = new char[sizeof(Node)];

  for (size_t i = 0; i < sizeof(Node); ++i)
    result[i] = '\0';

#define _FILL_DATA_(TYPE, MEMBER, VAL) *(TYPE *)&result[offsetof(Node, MEMBER)] = VAL;
  _FILL_DATA_(int32_t, zid, 0);
  //_FILL_DATA_(Node *, prev, nullptr);
  //_FILL_DATA_(Node *, next, nullptr);
  _FILL_DATA_(Node *, prevZ, nullptr);
  _FILL_DATA_(Node *, nextZ, nullptr);

  _FILL_DATA_(bool, steiner, false);
#undef _FILL_DATA_

  return result;
}

Node::Node(uint32_t id, double x, double y)
{
  static const char *const data = nodeFiller();
  memcpy(this, data, sizeof(Node));

  this->id = id;
  this->x  = x;
  this->y  = y;
}

//=======================================================
// Triangulator
//=======================================================

FistTriangulator::FistTriangulator(uint32_t pointCount, short dimension)
{
  _alloc     = std::move(RecyclableAllocator<Node, false>(pointCount));
  config.dim = dimension;
}

void FistTriangulator::SetBoundary(double *points, uint32_t size, Chiraty boundaryCharility)
{
  if (size < 2)
    return;

  _alloc.clear();
  _boundary = CreateLinkedList(points, size, CLOCKWISE, boundaryCharility);
}

void FistTriangulator::AppendHole(double *points, uint32_t size, Chiraty holeCharility /* = UNKNOWN */)
{
  if (!size || !_boundary)
    return;

  Node *node = CreateLinkedList(points, size, COUNTERCLOCKWISE, holeCharility);
  if (!node)
    return;
  if (node->next == node)
    node->steiner = true;

  // find the left-most point (node)
  Node *curNode = node, *leftmostNode = node;
  do
  {
    if ((curNode->x < leftmostNode->x) || (curNode->x == leftmostNode->x && curNode->y < leftmostNode->y))
      leftmostNode = curNode;
    curNode = curNode->next;
  } while (curNode != node);

  _holes.push_back(leftmostNode);
}

void FistTriangulator::Triangulate()
{
  if (!IsReady())
    return;

  triangles.reserve(triangles.size() + 3 * _alloc.size() + _holes.size() * 6 + 21);

  RemoveHoles();

  // eval bounding box, for z curve hashing
  if (config.useZCurveAccellaration > 0)
  {
    _minX = _maxX = _boundary->x;
    _minY = _maxY = _boundary->y;

    Node *curNode = _boundary->next;
    while (curNode != _boundary)
    {
      _minX = std::min(_minX, _boundary->x);
      _maxX = std::max(_maxX, _boundary->x);
      _minY = std::min(_minY, _boundary->y);
      _maxY = std::max(_maxY, _boundary->y);

      curNode = curNode->next;
    }

    _invSize = std::max(_maxX - _minX, _maxY - _minY);
    _invSize = _invSize < config.tolerance /* 0 or tol ? */ ? 32767. / _invSize : 0.;
  }

  Earcut(_boundary);
}

Node *FistTriangulator::CreateLinkedList(double *points,
                                         uint32_t size,
                                         Chiraty expectedChiraty,
                                         Chiraty targetChiraty)
{
  if (!size)
    return nullptr;

  Node *lastNode = nullptr;

  if (expectedChiraty != UNKNOWN && targetChiraty == UNKNOWN)
    targetChiraty = EvalSignedArea(points, size) > 0 ? CLOCKWISE : COUNTERCLOCKWISE;

  _alloc.reserve(_alloc.size() + size + 3);

  if (expectedChiraty == UNKNOWN || targetChiraty == expectedChiraty)
  {
    for (uint32_t i = 0; i < size; ++i)
      lastNode = InsertNode(i, points[config.dim * i], points[config.dim * i + 1], lastNode);
  }
  else
  {
    for (uint32_t i = size - 1; i != -1; --i)
      lastNode = InsertNode(i, points[config.dim * i], points[config.dim * i + 1], lastNode);
  }

  // fix tail
  if (lastNode && lastNode->x == lastNode->next->x && lastNode->y == lastNode->next->y)
  {
    lastNode = lastNode->next;
    RemoveNode(lastNode->prev);
  }

  return lastNode;
}

// David Eberly's Algorithm
// Ref: https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf
// todo: maybe searching through the whole polygon is not necessary. Use a geometric hashing technique
Node *FistTriangulator::FindBridge(Node *boundary, Node *hole) const
{
  Node *curBdrNode = _boundary, *nearestBdrEdgeNode = nullptr;
  const Node *leftMost = hole;

  /*        + cur
           /
          /       /
         o-------*
        /         \__
       /
      + next
  To find the edge with which the intersection of the horizontal line from the leftmost vertex of the hole has
  the largest x value smaller than the x coordinate of the vertex. Since the boundary polygon is CW, only
  edges going down can be candidates. */

  double maxX = -std::numeric_limits<double>::infinity();

  do
  {
    /* not going down or the horizontal line and the edge are not intersected */
    if (leftMost->y > curBdrNode->y || leftMost->y < leftMost->next->y ||
        curBdrNode->y == curBdrNode->next->y)
    {
      curBdrNode = curBdrNode->next;
      continue;
    }

    /* going down and intersected, x = x0 + (y - y0) / k */
    double ox = curBdrNode->x + (leftMost->y - curBdrNode->y) * (curBdrNode->next->x - curBdrNode->x) /
                                    (curBdrNode->next->y - curBdrNode->y);
    if (ox <= leftMost->x && ox > maxX)
    {
      maxX               = ox;
      nearestBdrEdgeNode = curBdrNode->x < curBdrNode->next->x /* left-top to right-bottom if true */
                               ? curBdrNode
                               : curBdrNode->next; /* select vertex with the lower x */
      if (std::abs(ox - leftMost->x) < config.tolerance / 4)
        /* leftmost vertex of the hole lies on the edge */
        return nearestBdrEdgeNode;
    }
    curBdrNode = curBdrNode->next;

  } while (curBdrNode != _boundary);

  if (!nearestBdrEdgeNode)
    return nullptr;

  // check if the diagonal is valid with no edge crossed. This is necessary since self-intersection may exist.
  /*      + cur
         /
        /       /
       o-------o                 o is the vertices of the triangle we focus on
      /.......  \__
     /....↖ any point in this triangle?
    /..
   o next
  If no such point, then the diagonal is valid, otherwise choose the vertex of the minimum angle with the ray
  as connection vertex */
  // clang-format off
  Node *originalNearestBdrEdgeNode = curBdrNode = nearestBdrEdgeNode;
  double minTan = std::numeric_limits<double>::infinity();
  // clang-format on

  do
  {
    // leftMost is the rightmost vertex of the triangle, nearestBdrEdgeNode is the leftmost
    if (curBdrNode->x >= leftMost->x
        /* current node is outside of the triangle since its x coordinate is larger than the rightmost vertex
           of the triangle */
        || curBdrNode->x < nearestBdrEdgeNode->x
        /* current node is outside of the triangle since its x coordinate is smaller than the leftmost vertex
           of the triangle */
        || !PointInTriangle(leftMost->y < nearestBdrEdgeNode->y ? leftMost->x : maxX, leftMost->y, /* A */
                            maxX, leftMost->y,                                                     /* B */
                            leftMost->y < nearestBdrEdgeNode->y ? maxX : leftMost->x, leftMost->y, /* C */
                            curBdrNode->x, curBdrNode->y                                           /* P */
                            )
        /* current node is outside of the triangle verified by a test method */
        /* triangle (A, B, C), test point (P)

           case 1:                                case 2:
             leftMost.y < nearestBdrEdgeNode.y      leftMost.y > nearestBdrEdgeNode.y

           o <- nearestBdrEdgeNode (C)                    +
            \...             ___                         /            ____
             \......        /                           /            /
              \.........   /                      maxX /            /
        (B) -> o----------o <- leftMost (A)    (B) -> o------------o <- leftMost (C)
           maxX \          \  Hole                   /...........   \   Hole
                 \          \___                    /........        \_____
                  \                                /....
                   \                              /.
                    +                            o <- nearestBdrEdgeNode (A)
        */
    )
    {
      curBdrNode = curBdrNode->next;
      continue;
    }

    // find a point P in the triangle, we cannot add the original diagonal, but to update the
    // nearestBdrEdgeNode
    double absTan = std::abs(hole->y - curBdrNode->y) / (hole->x - curBdrNode->x);
    if (LocallyInside(hole, curBdrNode)
        /* outside the hole */
        && (absTan < minTan
            /* better tangent */
            || (absTan == minTan &&
                (curBdrNode->x > nearestBdrEdgeNode->x ||
                 /* same tangent but nearer (x larger) */
                 (EvalSignedArea(nearestBdrEdgeNode->prev, nearestBdrEdgeNode, curBdrNode->prev) < 0 &&
                  EvalSignedArea(curBdrNode->next, nearestBdrEdgeNode, nearestBdrEdgeNode->next) < 0)
                 /*                  +
                                    /            _____
                                   ↗            /
                             maxX /            /
                          (B) -> o------------o <- leftMost (C)
                                /...........   \   Hole
                               ↗........        \_____
                              /....    + <- current.prev
                             /.   -
                 ---->------o <- nearestBdrEdgeNode (A)
                         ._
                       + <- current
                        \
                         \
                          \
                           + <- current->next
                The sector (ear) of `current` is contained in the sector of `nearestBdrEdgeNode`
                */
                 ))))
    {
      nearestBdrEdgeNode = curBdrNode;
      minTan             = absTan;
    }

    curBdrNode = curBdrNode->next;
  } while (curBdrNode != originalNearestBdrEdgeNode);

  return nearestBdrEdgeNode;
}

Node *FistTriangulator::SplitPolygon(Node *A, Node *B)
{
  /*
      \              /                   \             /
       \            /                     \  A      B /
        \ A      B /          -->          \_________/
        /          \                        __________
       /            \                      /          \
      /              \                    /            \
                                         /              \
  */
  Node *newA = InsertNode(A->id, A->x, A->y), *newB = InsertNode(B->id, B->x, B->y);

  newA->next = A->next;
  newA->prev = newB;
  newB->next = newA;
  newB->prev = B->prev;

  A->next->prev = newA;
  B->prev->next = newB;

  A->next = B;
  B->prev = A;

  return newB;
}

Node *FistTriangulator::RemoveHoles()
{
  std::sort(_holes.begin(), _holes.end(), [](Node *left, Node *right) -> bool { return left->x < right->x; });

  for (const auto leftMostNodeOfHole : _holes)
  {
    // try to find a bridge diagonal between the hole and the boundary
    Node *boundaryNode = FindBridge(_boundary, leftMostNodeOfHole);
    if (!boundaryNode)
      continue;

    // merge the hole to the boundary by adding a bridge between `boundaryNode` and `leftMostNodeOfHole`
    /*
                +
               /            ____
              /            /
             /____________/ <- returns this
            ______________
           /              \   Hole
          /                \_____
         /
        /
       o
    */
    Node *mergedBoundary = SplitPolygon(boundaryNode, leftMostNodeOfHole);

    // filter collinear points around the cuts
    _boundary = (RemoveRebundantVertices(mergedBoundary, mergedBoundary->next),
                 RemoveRebundantVertices(boundaryNode, boundaryNode->next));
  }

  return _boundary;
}

Node *FistTriangulator::RemoveRebundantVertices(Node *start, Node *end)
{
  // remove coincident or collinear points
  if (!end)
    end = start;  // scan the hole border

  Node *curNode     = start;
  bool furtherCheck = false;
  /* if end->next or further vertices are coincident with end, then do further check */
  do
  {
    furtherCheck = false;
    if (!curNode->steiner
        /* steiner point, which is provided by the user, should not be removed */
        && (curNode->x == curNode->next->x && curNode->y == curNode->next->y
            /* coincident */
            || EvalSignedArea(curNode->prev, curNode, curNode->next))
        /* collinear */
        )  // todo: tolerance?
    {
      RemoveNode(curNode);  // caution: don't overwrite curNode as its already removed.
      curNode = end = curNode->prev;

      if (curNode == curNode->next)
        break;
      furtherCheck = true;
    }
    else
    {
      curNode = curNode->next;
    }

  } while (furtherCheck || curNode != end);

  return end;
}

Node *FistTriangulator::CureLocalIntersections(Node *start)
{
  /*
         /                                         /
        /   * nextNext                            /  _* nextNext
       /     \                                   / _/
       *------+---------* curNode      ->        *
    prev       \       /                      prev
                \     /
                 \   /
                  \ /
                   *
                   next
  */
  Node *curNode = start;
  do
  {
    Node *prev = curNode->prev, *next = curNode->next, *nextNext = next->next;

    if ((prev->x == nextNext->x && prev->y == nextNext->y)
        /* prev and nextNext coincident */
        || !Intersected(prev, curNode, next, nextNext)
        /* not intersected at all */
        || !LocallyInside(prev, nextNext) || !LocallyInside(nextNext, prev)
        /* outside holes */
    )
    {
      curNode = curNode->next;
      continue;
    }

    for (Node *node : {prev, curNode, nextNext})
      triangles.push_back(node->id);

    RemoveNode(curNode);
    RemoveNode(curNode->next);

    start   = nextNext;
    curNode = start->next;
  } while (curNode != start);

  return RemoveRebundantVertices(start);
}

void FistTriangulator::ClumsyEarcut(Node *start)
{
  Node *curNode = start;
  do
  {
    Node *anotherNode = curNode->next->next;
    while (anotherNode != curNode->prev)
    {
      if (curNode->id == anotherNode->id || !ValidDiagonal(curNode, anotherNode))
      {
        anotherNode = anotherNode->next;
        continue;
      }

      start = RemoveRebundantVertices(curNode, curNode->next);

      Node *start2 = SplitPolygon(curNode, anotherNode);
      start2       = RemoveRebundantVertices(start2, start2->next);

      Earcut(start);
      Earcut(start2);

      return;
    }
    curNode = curNode->next;
  } while (curNode != start);
}

void FistTriangulator::Earcut(Node *start, int mode /* = 0 */)
{
  if (!start)
    return;
  if (mode == 0 && config.useZCurveAccellaration > 0)
    AssignZOrder(start);

  auto IsValidEar = std::bind(
      config.useZCurveAccellaration > 0 ? &FistTriangulator::ValidHashedEar : &FistTriangulator::ValidEar,
      this, std::placeholders::_1);

  Node *curNode = start, *prev = nullptr, *next = nullptr;

  // iterate through ears, slicing them one by one
  while (curNode->prev != curNode->next)
  {
    prev = curNode->prev;
    next = curNode->next;

    if (IsValidEar(curNode))
    {
      // cut off the triangle
      triangles.emplace_back(prev->id);
      triangles.emplace_back(curNode->id);
      triangles.emplace_back(next->id);

      RemoveNode(curNode);

      // skipping the next vertex leads to less sliver triangles
      curNode = start = next->next;
      continue;
    }

    curNode = next;

    // if we looped through the whole remaining polygon and can't find any more ears
    if (curNode == start)
    {
      // try filtering points and slicing again
      if (mode == 0)
      {
        curNode = RemoveRebundantVertices(curNode);
        Earcut(curNode, 1);
      }
      // if this didn't work, try curing all small self-intersections locally
      else if (mode == 1)
      {
        curNode = RemoveRebundantVertices(curNode);
        curNode = CureLocalIntersections(curNode);
        Earcut(curNode, 2);
      }
      // as a last resort, try splitting the remaining polygon into two
      else if (mode == 2)
      {
        ClumsyEarcut(curNode);
      }
      break;
    }
  }
}

double FistTriangulator::EvalSignedArea(double *points, uint32_t size) const
{
  double area = 0.;
  for (uint32_t i = 0, j = size - 1 /* last */; i < size; j = i, ++i)
  {
    double x0 = points[config.dim * j], y0 = points[config.dim * j + 1], x1 = points[config.dim * i],
           y1 = points[config.dim * i + 1];
    area += (x0 - x1) * (y1 + y0);
  }
  return area;
}

double FistTriangulator::EvalSignedArea(const Node *const A, const Node *const B, const Node *const C)
{
  return (B->y - A->y) * (C->x - B->x) - (B->x - A->x) * (C->y - B->y);
}

bool FistTriangulator::LocallyInside(const Node *const base, const Node *const point)
{
  bool cw = EvalSignedArea(base->prev, base, base->next) < 0;

  /* true occasions:
    [area < 0]                  [area > 0]
     base        base->next                    base->next
        +====>====+                             ++..........
       /...........                Both > 0    //...........
      ↗............                           ↗↗............ <- Area(base, base->next, point) < 0
     /........* <- point                     //.............
    +..............            +=======>====*- - - - - - - -
    base->prev             base->prev....../.base...........
           ↑                   ............................. <- Both < 0
          Both > 0             ........../..................
                                  ↑
                                  Area(base, point, base->prev) < 0
  */
  if (cw)
    return EvalSignedArea(base, point, base->next) >= 0 && EvalSignedArea(base, base->prev, point) >= 0;
  return EvalSignedArea(base, point, base->prev) < 0 || EvalSignedArea(base, base->next, point) < 0;
}

bool FistTriangulator::MiddleInside(const Node *const A, const Node *const B)
{
  // test if a point (which is the midpoint of A and B) is in a polygon by counting the segments on its right
  double midX = (A->x + B->x) / 2, midY = (A->y + B->y) / 2;
  uint32_t depth = EvalDepth(A, midX, midY);

  return depth & 0b1; /* odd depth indicates internally */
}

bool FistTriangulator::ValidDiagonal(const Node *A, const Node *B) const
{
  if (A->next->id == B->id || A->prev->id == B->id) /* is border segment */
    return false;
  if (IntersectedWithPolygon(A, B, A))
    return false;

  bool locallyVisibleChecked = LocallyInside(A, B) && LocallyInside(B, A) && MiddleInside(A, B) &&
                               /* does not create opposite-facing sectors */
                               (EvalSignedArea(A, B->prev, B) != 0. || EvalSignedArea(A, B->prev, B) != 0.);
  if (locallyVisibleChecked)
    return true;

  bool zeroLengthChecked = A->x == B->x && A->y == B->y && EvalSignedArea(A->prev, A, A->next) &&
                           EvalSignedArea(B->prev, B, B->next);
  return zeroLengthChecked;

  return false;
}

uint32_t FistTriangulator::EvalDepth(const Node *const base, double x, double y)
{
  const Node *curNode = base;
  uint32_t depth      = 0;
  do
  {
    if ((curNode->y > y) != (curNode->next->y > y)
        /* the horizontal line from (x, y) intersects the segment from curNode to curNode->next */
        && (x <
            curNode->x + (curNode->next->x - curNode->x) * (y - curNode->y) / (curNode->next->y - curNode->y))
        /* the segment lies on the right of (x, y). Certainly, we can also count the segments on the left by
           turning < to > */
    )
    {
      ++depth;
    }
  } while (curNode != base);

  return depth;
}

bool FistTriangulator::PointInTriangle(double Ax,
                                       double Ay,
                                       double Bx,
                                       double By,
                                       double Cx,
                                       double Cy,
                                       double Px,
                                       double Py)
{
  // todo: the triangle is CW?
  bool res = (Cx - Px) * (Ay - Py) >= (Ax - Px) * (Cy - Py) &&
             (Ax - Px) * (By - Py) >= (Bx - Px) * (Ay - Py) && (Bx - Px) * (Cy - Py) >= (Cx - Px) * (By - Py);
  return res;
}

bool FistTriangulator::Intersected(const Node *L11, const Node *L12, const Node *L21, const Node *L22)
{
  /* positive: 1, zero: 0, negative: -1 */
  static auto GetSign = [](double val) -> int { return (val > 0.) - (val < 0.); };
  /* check if B is on A,C, knowing that A, B, C collinear */
  static auto PointOnSegment = [](const Node *A, const Node *B, const Node *C) -> bool {
    bool xChecked = B->x <= std::max<double>(A->x, C->x) && B->x >= std::min<double>(A->x, C->x);
    bool yChecked = B->y <= std::max<double>(A->y, C->y) && B->y >= std::min<double>(A->y, C->y);
    return xChecked && yChecked;
  };

  int o1 = GetSign(EvalSignedArea(L11, L12, L21));
  int o2 = GetSign(EvalSignedArea(L11, L12, L22));
  int o3 = GetSign(EvalSignedArea(L21, L22, L11));
  int o4 = GetSign(EvalSignedArea(L21, L22, L12));

  if (o1 != o2 && o3 != o4)
    return true;  // general case

  if ((o1 == 0 && PointOnSegment(L11, L21, L12)) || (o2 == 0 && PointOnSegment(L11, L22, L12)) ||
      (o3 == 0 && PointOnSegment(L21, L11, L22)) || (o4 == 0 && PointOnSegment(L21, L12, L22)))
    return true;  // collinear

  return false;
}

bool FistTriangulator::IntersectedWithPolygon(const Node *A, const Node *B, const Node *polygon)
{
  const Node *curNode = polygon;
  do
  {
    bool isAOrB = curNode->id == A->id || curNode->next->id == A->id || curNode->id == B->id ||
                  curNode->next->id == B->id; /* use id to avoid added diagonal when connecting holes */
    if (!isAOrB && Intersected(curNode, curNode->next, A, B))
      return true;

    curNode = curNode->next;
  } while (curNode != polygon);

  return false;
}

bool FistTriangulator::ValidEar(const Node *node) const
{
  const Node *prev = node->prev, *next = node->next;

  if (EvalSignedArea(prev, node, next) >= 0) /* reflex */
    return false;

  // now make sure we don't have other points inside the potential ear
  const Node *curNode = next->next;
  while (curNode != node->prev)
  {
    if (PointInTriangle(prev->x, prev->y, node->x, node->y, next->x, next->y, curNode->x, curNode->y) &&
        EvalSignedArea(curNode->prev, curNode, curNode->next) >= 0)
      return false;
    curNode = curNode->next;
  }

  return true;
}

bool FistTriangulator::ValidHashedEar(const Node *node) const
{
  const Node *const prev = node->prev, *const next = node->next;

  if (EvalSignedArea(prev, node, next) >= 0) /* reflex */
    return false;

  // triangle bbox; min & max are calculated like this for speed
  const double minTX = std::min(prev->x, std::min(node->x, next->x));
  const double minTY = std::min(prev->y, std::min(node->y, next->y));
  const double maxTX = std::max(prev->x, std::max(node->x, next->x));
  const double maxTY = std::max(prev->y, std::max(node->y, next->y));

  // z-order range for the current triangle bbox;
  const int32_t minZ = EvalZOrder(minTX, minTY);
  const int32_t maxZ = EvalZOrder(maxTX, maxTY);

  // first look for points inside the triangle in increasing z-order
  const Node *curZNode = node->nextZ;

  while (curZNode && curZNode->zid <= maxZ)
  {
    if (curZNode != node->prev && curZNode != node->next &&
        PointInTriangle(prev->x, prev->y, node->x, node->y, next->x, next->y, curZNode->x, curZNode->y) &&
        EvalSignedArea(curZNode->prev, curZNode, curZNode->next) >= 0)
      return false;
    curZNode = curZNode->nextZ;
  }

  // then look for points in decreasing z-order
  curZNode = node->prevZ;

  while (curZNode && curZNode->zid >= minZ)
  {
    if (curZNode != node->prev && curZNode != node->next &&
        PointInTriangle(prev->x, prev->y, node->x, node->y, next->x, next->y, curZNode->x, curZNode->y) &&
        EvalSignedArea(curZNode->prev, curZNode, curZNode->next) >= 0)
      return false;
    curZNode = curZNode->prevZ;
  }

  return true;
}

int32_t FistTriangulator::EvalZOrder(double x, double y) const
{
  // Ref: https://en.wikipedia.org/wiki/Z-order_curve
  int32_t ix = static_cast<int32_t>((x - _minX) * _invSize);
  int32_t iy = static_cast<int32_t>((y - _minY) * _invSize);

  ix = (ix | (ix << 8)) & 0x00FF00FF;
  ix = (ix | (ix << 4)) & 0x0F0F0F0F;
  ix = (ix | (ix << 2)) & 0x33333333;
  ix = (ix | (ix << 1)) & 0x55555555;

  iy = (iy | (iy << 8)) & 0x00FF00FF;
  iy = (iy | (iy << 4)) & 0x0F0F0F0F;
  iy = (iy | (iy << 2)) & 0x33333333;
  iy = (iy | (iy << 1)) & 0x55555555;

  return ix | (iy << 1);
}

void FistTriangulator::AssignZOrder(Node *borderNode)
{
  if (!borderNode)
    return;

  Node *curNode = borderNode;
  // init
  do
  {
    if (!curNode->zid)
      curNode->zid = EvalZOrder(curNode->x, curNode->y);

    curNode->prevZ = curNode->prev;
    curNode->nextZ = curNode->next;

    curNode = curNode->next;
  } while (curNode != borderNode);

  // cut tail
  curNode->prevZ = curNode->prevZ->nextZ = nullptr;

  // sort
  // Simon Tatham's linked list merge sort algorithm
  // Ref: http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
  Node *P, *Q, *E, *tail;
  int i, numMerges, PSize, QSize;
  int inSize = 1;
  while (true)
  {
    P         = curNode;
    curNode   = nullptr;
    tail      = nullptr;
    numMerges = 0;

    while (P)
    {
      numMerges++;
      Q     = P;
      PSize = 0;
      for (i = 0; i < inSize; i++)
      {
        PSize++;
        Q = Q->nextZ;
        if (!Q)
          break;
      }

      QSize = inSize;

      while (PSize > 0 || (QSize > 0 && Q))
      {

        if (PSize == 0)
        {
          E = Q;
          Q = Q->nextZ;
          QSize--;
        }
        else if (QSize == 0 || !Q)
        {
          E = P;
          P = P->nextZ;
          PSize--;
        }
        else if (P->zid <= Q->zid)
        {
          E = P;
          P = P->nextZ;
          PSize--;
        }
        else
        {
          E = Q;
          Q = Q->nextZ;
          QSize--;
        }

        if (tail)
          tail->nextZ = E;
        else
          curNode = E;

        E->prevZ = tail;
        tail     = E;
      }

      P = Q;
    }

    tail->nextZ = nullptr;

    if (numMerges <= 1)
      break;

    inSize *= 2;
  }
}

Node *FistTriangulator::InsertNode(uint32_t id, double x, double y, Node *last)
{
  // original: last --> last.next
  // functioned: last --> node --> last.next

  Node *node = _alloc.alloc(id, x, y);

  // no last node provided
  if (!last)
  {
    node->prev = node->next = node;
    return node;
  }

  // insert after last
  node->next       = last->next;
  node->prev       = last;
  node->next->prev = node;
  last->next       = node;

  return node;
}

void FistTriangulator::RemoveNode(Node *node)
{
  // detach
  node->next->prev = node->prev;
  node->prev->next = node->next;

  if (node->prevZ)
    node->prevZ->nextZ = node->nextZ;
  if (node->nextZ)
    node->nextZ->prevZ = node->prevZ;

  _alloc.remove(node);
}